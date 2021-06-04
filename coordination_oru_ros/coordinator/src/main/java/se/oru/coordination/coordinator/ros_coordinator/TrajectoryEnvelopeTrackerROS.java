package se.oru.coordination.coordinator.ros_coordinator;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Quaternion;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.MessageListener;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Subscriber;
import org.ros.node.topic.Publisher;

import orunav_msgs.CoordinatorTime;
import orunav_msgs.CoordinatorTimeVec;
import orunav_msgs.DeltaTVec;
import orunav_msgs.ExecuteTask;
import orunav_msgs.ExecuteTaskRequest;
import orunav_msgs.ExecuteTaskResponse;
import orunav_msgs.Operation;
import orunav_msgs.Task;
import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.TrackingCallback;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;

public class TrajectoryEnvelopeTrackerROS extends AbstractTrajectoryEnvelopeTracker {

	protected static int goalID = 0;
	protected ConnectedNode node = null;
	protected RobotReport currentRR = null;
	protected Boolean isBraking = null;
	protected Subscriber<orunav_msgs.RobotReport> subscriber = null;
	protected Task currentTask = null;
	protected VEHICLE_STATE currentVehicleState = null;
	boolean calledExecuteFirstTime = false;
	private double prevDistance = 0.0;
	private long lastUpdateTime = -1;
	int prevSeqNumber = -1;

	public static enum VEHICLE_STATE {_IGNORE_, WAITING_FOR_TASK, PERFORMING_START_OPERATION, DRIVING, PERFORMING_GOAL_OPERATION, TASK_FAILED, WAITING_FOR_TASK_INTERNAL, DRIVING_SLOWDOWN, AT_CRITICAL_POINT, BRAKE}
		
	Publisher<orunav_msgs.Task> task_pub_;
	
	public TrajectoryEnvelopeTrackerROS(TrajectoryEnvelope te, double temporalResolution, TrajectoryEnvelopeCoordinator tec, TrackingCallback cb, ConnectedNode connectedNode, Task currentTask) {
		super(te, temporalResolution, tec, tec.getRobotTrackingPeriodInMillis(te.getRobotID()), cb);
		this.node = connectedNode;
		this.currentTask = currentTask;
		
		final TrajectoryEnvelopeTrackerROS thisTracker = this;

		String reportTopic = "report";
		///
		ParameterTree params = node.getParameterTree();
		try {
			reportTopic = params.getString("/" + node.getName() + "/report_topic", "report");
		}
		catch (org.ros.exception.ParameterNotFoundException e) {
			System.out.println("== Parameter not found ==");
			e.printStackTrace();
		}
		///

		// mfc: exposed to 
		task_pub_ = node.newPublisher("robot"+te.getRobotID()+"/control/task", orunav_msgs.Task._TYPE);
		task_pub_.setLatchMode(true);
		
		if (currentTask == null) throw new Error("Trying to instantiate a TrajectoryEnvelopeTrackerROS for Robot" + te.getRobotID() + " with currentTask == " + currentTask);
		subscriber = connectedNode.newSubscriber("robot"+te.getRobotID()+"/"+reportTopic, orunav_msgs.RobotReport._TYPE);
	    subscriber.addMessageListener(new MessageListener<orunav_msgs.RobotReport>() {
	      @Override
	      public void onNewMessage(orunav_msgs.RobotReport message) {
	    	  synchronized(thisTracker.getTrajectoryEnvelope()) {
		    	  TrajectoryEnvelope thisTE = thisTracker.getTrajectoryEnvelope();
		    	  if (lastUpdateTime == -1) lastUpdateTime = getCurrentTimeInMillis();
		    	  currentVehicleState = VEHICLE_STATE.values()[message.getStatus()];
		    	  Quaternion quat = new Quaternion(message.getState().getPose().getOrientation().getX(), message.getState().getPose().getOrientation().getY(), message.getState().getPose().getOrientation().getZ(), message.getState().getPose().getOrientation().getW());
		    	  Pose pose = new Pose(message.getState().getPose().getPosition().getX(), message.getState().getPose().getPosition().getY(), quat.getTheta());
		    	  int index = Math.min(message.getSequenceNum(), thisTE.getTrajectory().getPoseSteering().length-1);
	    		  long currentTime = getCurrentTimeInMillis(); 
	    		  double vel = -1.0;
	    		  double newDistance = -1.0;
		    	  if (currentVehicleState.equals(VEHICLE_STATE.PERFORMING_GOAL_OPERATION)) index = thisTracker.getTrajectoryEnvelope().getTrajectory().getPoseSteering().length-1;
		    	  else {
		    		  //Need to estimate velocity and distance traveled for use in the FW model...
		    		  Trajectory traj = thisTE.getTrajectory();
		    		  newDistance = 0;
		    		  for (int i = 0; i < index-1; i++) newDistance += traj.getPose()[i].distanceTo(traj.getPose()[i+1]);
		    		  long deltaT = currentTime-lastUpdateTime;
		    		  vel = (newDistance-prevDistance)/(deltaT/1000.0);
		    	  }
	    		  currentRR = new RobotReport(thisTE.getRobotID(), pose, index, vel, newDistance, -1);
	    		  isBraking = new Boolean(message.getNbActiveBrakeReasons() > 0);
	    		  lastUpdateTime = currentTime;
	    		  prevDistance = newDistance;
		    	  metaCSPLogger.info("Current state of robot" + thisTE.getRobotID() + ": " + currentVehicleState + ", received index: " + message.getSequenceNum() + " [in internal report:" + currentRR.getPathIndex() + "]");
		    	  onPositionUpdate();
		      }
	      }
	    });
	    
	    calledExecuteFirstTime = false;
	}
	
	public Boolean isBraking() {
		return isBraking;
	}
	
	public VEHICLE_STATE getVehicleState() {
		while (currentVehicleState == null) {
			try { Thread.sleep(100); }
			catch (InterruptedException e) { e.printStackTrace(); }
		}
		return currentVehicleState;
	}
	
	@Override
	public long getCurrentTimeInMillis() {
		return TimeUnit.NANOSECONDS.toMillis(node.getCurrentTime().totalNsecs());
	}

	@Override
	public RobotReport getRobotReport() {
		while (currentRR == null) {
			try { Thread.sleep(100); }
			catch (InterruptedException e) { e.printStackTrace(); }
		}
		return currentRR;
	}
	
	@Override
	public void setCriticalPoint(int arg0) {
		callExecuteTaskService(arg0, calledExecuteFirstTime);
		calledExecuteFirstTime = true;
		setCanStartTracking();
	}
	
	@Override
	public void startTracking() { }
	
	@Override
	protected void finishTracking() {
		super.finishTracking();
		subscriber.shutdown();
	}
	
	@Override
	protected void startMonitoringThread() {
		
		//Start a thread that monitors the sub-envelopes and finishes them when appropriate
		Thread monitorSubEnvelopes = new Thread("Abstract tracker " + te.getComponent()) {
			@Override
			public void run() {	

				if (cb != null) cb.beforeTrackingStart();

				//Monitor the sub-envelopes...
				while (true) {
					
					synchronized(tec.getSolver()) { 
					//Track if past start time
					if (te.getTemporalVariable().getEST() <= getCurrentTimeInMillis()) {
		
							if (cb != null && !calledOnTrackingStart) {
								calledOnTrackingStart = true;
								cb.onTrackingStart();
							}
							
							if (!calledStartTracking) {
								calledStartTracking = true;							
								startTracking();
							}
	
							//if (!startedGroundEnvelopes.isEmpty()) printStartedGroundEnvelopes();
							RobotReport rr = null;
							while ((rr = tec.getRobotReport(te.getRobotID())) == null) {
								metaCSPLogger.info("(waiting for "+te.getComponent()+"'s tracker to come online)");
								try { Thread.sleep(100); }
								catch (InterruptedException e) { e.printStackTrace(); }
							}
	
							//Get current sequence number from robot report...
							int currentSeqNumber = rr.getPathIndex();
		
							//Get all ground envelopes of this super-envelope that are not finished (except the last one)...
							for (TrajectoryEnvelope subEnv : getAllSubEnvelopes()) {
								if (subEnv.hasSuperEnvelope()) {
									if (subEnv.getSequenceNumberStart() <= currentSeqNumber && !startedGroundEnvelopes.contains(subEnv)) {
										startedGroundEnvelopes.add(subEnv);
										metaCSPLogger.info(">>>> Dispatched (ground envelope) " + subEnv);
										if (cb != null) cb.onNewGroundEnvelope();
									}
									if (subEnv.getSequenceNumberEnd() < currentSeqNumber && !finishedGroundEnvelopes.contains(subEnv)) {
										finishedGroundEnvelopes.add(subEnv);
										metaCSPLogger.info("<<<< Finished (ground envelope) " + subEnv);
										if (subEnv.getSequenceNumberEnd() < te.getSequenceNumberEnd()) fixDeadline(subEnv, 0);
									}
									else if (!finishedGroundEnvelopes.contains(subEnv) && currentSeqNumber > prevSeqNumber) {
										updateDeadline(subEnv, 0);
									}
								}							
							}
						
							//Stop when last path point reached (or we missed that report and the path point is now 0)
							if (te.getSequenceNumberEnd() == currentSeqNumber || (currentSeqNumber < prevSeqNumber && currentSeqNumber <= 0)) {
								if (currentVehicleState != null && currentVehicleState.equals(VEHICLE_STATE.WAITING_FOR_TASK)) {
									metaCSPLogger.info("At last path point (current: " + currentSeqNumber + ", prev: " + prevSeqNumber + ") of " + te + "...");
									for (TrajectoryEnvelope toFinish : startedGroundEnvelopes) {
										if (!finishedGroundEnvelopes.contains(toFinish)) {
											metaCSPLogger.info("<<<< Finished (ground envelope) " + toFinish);
											finishedGroundEnvelopes.add(toFinish);
										}
									}
									break;
								}
							}
							//Update previous seq number
							prevSeqNumber = currentSeqNumber;
						}
					}
				
					//Sleep a little...
					try { Thread.sleep(trackingPeriodInMillis); }
					catch (InterruptedException e) { e.printStackTrace(); }

				}

				synchronized(tec.getSolver()) { 
					if (cb != null) cb.beforeTrackingFinished();
					finishTracking();
					if (cb != null) cb.onTrackingFinished();
				}
				
			}
		};
		
		monitorSubEnvelopes.start();
	}
		
	public void setOperations(String startOperation, String goalOperation) throws IllegalArgumentException, IllegalAccessException {
		Operation startOp = node.getTopicMessageFactory().newFromType(Operation._TYPE);
		Operation goalOp = node.getTopicMessageFactory().newFromType(Operation._TYPE);
		Class<Operation> c = Operation.class;
		int sop = Operation.NO_OPERATION;
		int gop = Operation.NO_OPERATION;
		
		for (Field f : c.getDeclaredFields()) {
			if (startOperation != null && f.getName().equals(startOperation)) sop = f.getInt(null);
			if (goalOperation != null && f.getName().equals(goalOperation)) gop = f.getInt(null);
		}
		
		startOp.setOperation(sop);
		currentTask.getTarget().setStartOp(startOp);
		goalOp.setOperation(gop);
		currentTask.getTarget().setGoalOp(goalOp);
	}
	
	public void setOperations(Operation startOp, Operation goalOp) {
		currentTask.getTarget().setStartOp(startOp);
		currentTask.getTarget().setGoalOp(goalOp);
	}
	
	private void callExecuteTaskService(int cp, boolean update) {

		ServiceClient<ExecuteTaskRequest, ExecuteTaskResponse> serviceClient;
		try {
			System.out.println("-------> Going to call service: /robot" + currentTask.getTarget().getRobotId() + "/execute_task");
			if (update)
			{
				System.out.println("-------> UPDATING TASK");
			}
			System.out.println("----------> and my TE is " + te);
			serviceClient = node.newServiceClient("/robot" + currentTask.getTarget().getRobotId() + "/execute_task", ExecuteTask._TYPE);
		}
		catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
		final ExecuteTaskRequest request = serviceClient.newMessage();
		CoordinatorTimeVec cts = computeCTsFromDTs(currentTask.getDts());
		currentTask.setCts(cts);		
		currentTask.setUpdate(update);
		currentTask.setCriticalPoint(cp);
		request.setTask(currentTask);

		serviceClient.call(request, new ServiceResponseListener<ExecuteTaskResponse>() {
			@Override
			public void onSuccess(ExecuteTaskResponse response) {
					System.out.println("Started execution of goal " + currentTask.getTarget().getGoalId() + " for robot " + currentTask.getTarget().getRobotId());
					task_pub_.publish(currentTask);
			}
			@Override
			public void onFailure(RemoteException arg0) {
				System.out.println("Failed to start execution of goal " + currentTask.getTarget().getGoalId() + " for robot " + currentTask.getTarget().getRobotId());
			}
		});		
		
	}
	
	private CoordinatorTimeVec computeCTsFromDTs(DeltaTVec dts) {
		CoordinatorTimeVec cts = node.getTopicMessageFactory().newFromType(CoordinatorTimeVec._TYPE);
		cts.setGoalId(dts.getGoalId());
		cts.setId(dts.getTrajId());
		ArrayList<CoordinatorTime> ctList = new ArrayList<CoordinatorTime>();
//		double currentTime = node.getCurrentTime().toSeconds();
//		double[] fastDTs = dts.getDts().get(0).getDt();
//		double[] slowDTs = dts.getDts().get(1).getDt();
//		double[] fastCTs = new double[fastDTs.length];
//		double[] slowCTs = new double[slowDTs.length];
		CoordinatorTime ctFast = node.getTopicMessageFactory().newFromType(CoordinatorTime._TYPE);
		CoordinatorTime ctSlow = node.getTopicMessageFactory().newFromType(CoordinatorTime._TYPE);
//		fastCTs[0] = currentTime;
//		slowCTs[0] = currentTime;
//		for (int i = 1; i < fastDTs.length; i++) {
//			fastCTs[i] = fastDTs[i]+fastCTs[i-1];
//		    slowCTs[i] = slowDTs[i]+slowCTs[i-1];
//		}
//		ctFast.setT(fastCTs);
//		ctSlow.setT(slowCTs);
		ctFast.setT(new double[] {-1, -1});
		ctSlow.setT(new double[] {-1, -1});
		ctList.add(ctFast);
		ctList.add(ctSlow);
		cts.setTs(ctList);
		return cts;
	}

	@Override
	protected void onTrajectoryEnvelopeUpdate() {
		synchronized(te) {
			PoseSteering[] newPS = te.getTrajectory().getPoseSteering();
			ArrayList<orunav_msgs.PoseSteering> newPath = new ArrayList<orunav_msgs.PoseSteering>();
			for (PoseSteering ps : newPS) {
				orunav_msgs.PoseSteering onePS = node.getTopicMessageFactory().newFromType(orunav_msgs.PoseSteering._TYPE);
				geometry_msgs.Pose oneP = node.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
				geometry_msgs.Point onePnt = node.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
				onePnt.setX(ps.getX());
				onePnt.setY(ps.getY());
				onePnt.setZ(0.0);
				oneP.setPosition(onePnt);
				Quaternion quat = new Quaternion(ps.getTheta());
				geometry_msgs.Quaternion oneQ = node.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);
				oneQ.setX(quat.getX());
				oneQ.setY(quat.getY());
				oneQ.setZ(quat.getZ());
				oneQ.setW(quat.getW());
				oneP.setOrientation(oneQ);
				onePS.setPose(oneP);
				onePS.setSteering(ps.getSteering());
				newPath.add(onePS);
			}
			currentTask.getPath().setPath(newPath);
			System.out.println("%%%% Going to send new PATH OF SIZE to robot " + te.getRobotID() + ": " + currentTask.getPath().getPath().size());
	
			//Reset the previous sequence number and flag
			prevSeqNumber = -1;
		}
	}
}
