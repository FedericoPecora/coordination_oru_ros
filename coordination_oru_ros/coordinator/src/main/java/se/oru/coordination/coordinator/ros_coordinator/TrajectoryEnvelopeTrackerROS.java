package se.oru.coordination.coordinator.ros_coordinator;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.Quaternion;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.MessageListener;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Subscriber;

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
	protected Subscriber<orunav_msgs.RobotReport> subscriber = null;
	protected Task currentTask = null;
	protected VEHICLE_STATE currentVehicleState = null;
	boolean waitingForGoalOperation = false;
	boolean calledExecuteFirstTime = false;

	public static enum VEHICLE_STATE {_IGNORE_, WAITING_FOR_TASK, PERFORMING_START_OPERATION, DRIVING, PERFORMING_GOAL_OPERATION, TASK_FAILED, WAITING_FOR_TASK_INTERNAL, DRIVING_SLOWDOWN, AT_CRITICAL_POINT}
	
	public TrajectoryEnvelopeTrackerROS(final TrajectoryEnvelope te, double temporalResolution, TrajectoryEnvelopeCoordinator tec, TrackingCallback cb, ConnectedNode connectedNode, Task currentTask) {
		super(te, temporalResolution, tec, 30, cb);
		this.node = connectedNode;
		this.currentTask = currentTask;
		if (currentTask == null) throw new Error("Trying to instantiate a TrajectoryEnvelopeTrackerROS for Robot" + te.getRobotID() + " with currentTask == " + currentTask);
		subscriber = connectedNode.newSubscriber("robot"+te.getRobotID()+"/report", orunav_msgs.RobotReport._TYPE);
	    subscriber.addMessageListener(new MessageListener<orunav_msgs.RobotReport>() {
	      @Override
	      public void onNewMessage(orunav_msgs.RobotReport message) {
	    	  Quaternion quat = new Quaternion(message.getState().getPose().getOrientation().getX(), message.getState().getPose().getOrientation().getY(), message.getState().getPose().getOrientation().getZ(), message.getState().getPose().getOrientation().getW());
	    	  Pose pose = new Pose(message.getState().getPose().getPosition().getX(), message.getState().getPose().getPosition().getY(), quat.getTheta());
	    	  int index = message.getSequenceNum();
	    	  if (waitingForGoalOperation) {
	    		  metaCSPLogger.info("Current state of robot" + te.getRobotID() + ": " + currentVehicleState);
	    		  currentRR = new RobotReport(te.getRobotID(), pose, te.getTrajectory().getPose().length-1, -1.0, -1.0, -1);
	    	  }
	    	  else currentRR = new RobotReport(te.getRobotID(), pose, index, -1.0, -1.0, -1);
	    	  currentVehicleState = VEHICLE_STATE.values()[message.getStatus()];
	    	  onPositionUpdate();
	    	  
	      }
	    });
	    
	    calledExecuteFirstTime = false;
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
	}

	@Override
	public void startTracking() { }
	
	@Override
	protected void finishTracking() {
		waitingForGoalOperation = true;
		while (currentVehicleState != null && (!currentVehicleState.equals(VEHICLE_STATE.WAITING_FOR_TASK))) {
			try { Thread.sleep(100); }
			catch (InterruptedException e) { e.printStackTrace(); }
		}
		super.finishTracking();
		subscriber.shutdown();
	}
	
	private void callExecuteTaskService(int cp, boolean update) {

		ServiceClient<ExecuteTaskRequest, ExecuteTaskResponse> serviceClient;
		try { serviceClient = node.newServiceClient("/robot" + currentTask.getTarget().getRobotId() + "/execute_task", ExecuteTask._TYPE); }
		catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
		final ExecuteTaskRequest request = serviceClient.newMessage();
		CoordinatorTimeVec cts = computeCTsFromDTs(currentTask.getDts());
		currentTask.setCts(cts);
		
		//Operations used by the current execution service
		Operation startOp = node.getTopicMessageFactory().newFromType(Operation._TYPE);
		startOp.setOperation(Operation.NO_OPERATION);
		currentTask.getTarget().setStartOp(startOp);
		Operation goalOp = node.getTopicMessageFactory().newFromType(Operation._TYPE);
		goalOp.setOperation(Operation.NO_OPERATION);
		currentTask.getTarget().setGoalOp(goalOp);
		
		currentTask.setUpdate(update);
		currentTask.setCriticalPoint(cp);
		request.setTask(currentTask);

		serviceClient.call(request, new ServiceResponseListener<ExecuteTaskResponse>() {
			@Override
			public void onSuccess(ExecuteTaskResponse response) {
					System.out.println("Started execution of goal " + currentTask.getTarget().getGoalId() + " for robot " + currentTask.getTarget().getRobotId());
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


}
