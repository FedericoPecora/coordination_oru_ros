package se.oru.coordination.coordinator.ros_coordinator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import org.apache.commons.lang.ArrayUtils;
import org.jgrapht.alg.connectivity.ConnectivityInspector;
import org.jgrapht.graph.SimpleDirectedGraph;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Quaternion;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Publisher;

import com.vividsolutions.jts.geom.Geometry;

import orunav_msgs.Task;
import orunav_msgs.BrakeTask;
import orunav_msgs.BrakeTaskRequest;
import orunav_msgs.BrakeTaskResponse;
import orunav_msgs.ReplanStatus;
import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.Dependency;
import se.oru.coordination.coordination_oru.TrackingCallback;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeTrackerDummy;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordinator.ros_coordinator.TrajectoryEnvelopeTrackerROS.VEHICLE_STATE;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;

public class TrajectoryEnvelopeCoordinatorROS extends TrajectoryEnvelopeCoordinator {

	protected ConnectedNode node = null;
	protected HashMap<Integer, orunav_msgs.Task> currentTasks = new HashMap<Integer, orunav_msgs.Task>();
	protected ConcurrentHashMap<Integer, AtomicBoolean> canReplan = new ConcurrentHashMap<Integer,AtomicBoolean>();
	protected ConcurrentHashMap<Integer,Integer> currentBreakingPathIndex = new ConcurrentHashMap<Integer,Integer>();
	protected Publisher<orunav_msgs.ReplanStatus> replanStatusPublisher = null;

	public TrajectoryEnvelopeTrackerROS getCurrentTracker(int robotID) {
		return (TrajectoryEnvelopeTrackerROS)this.trackers.get(robotID);
	}
		
	public TrajectoryEnvelopeCoordinatorROS(int CONTROL_PERIOD, double TEMPORAL_RESOLUTION, final ConnectedNode connectedNode) {
		super(CONTROL_PERIOD, TEMPORAL_RESOLUTION);
		this.node = connectedNode;
		replanStatusPublisher = node.newPublisher("/coordinator/replan/status", orunav_msgs.ReplanStatus._TYPE);
		replanStatusPublisher.setLatchMode(true);
	}

	public TrajectoryEnvelopeCoordinatorROS(final ConnectedNode connectedNode) {
		this(1000, 1000.0, connectedNode);
	}

	@Override
	public long getCurrentTimeInMillis() {
		return TimeUnit.NANOSECONDS.toMillis(node.getCurrentTime().totalNsecs());
	}
	
	public void setCurrentTask(int robotID, Task currentTask) {
		System.out.println("SET TASK (robotID,currentTask): (" + robotID + "," + currentTask + ")");
		this.currentTasks.put(robotID, currentTask);
	}

	public orunav_msgs.Task getCurrentTask(int robotID) {
		if (this.currentTasks.containsKey(robotID)) return this.currentTasks.get(robotID);
		return null;
	}

	@Override
	public AbstractTrajectoryEnvelopeTracker getNewTracker(TrajectoryEnvelope te, TrackingCallback cb) {
		TrajectoryEnvelopeTrackerROS tet = new TrajectoryEnvelopeTrackerROS(te, this.TEMPORAL_RESOLUTION, this, cb, this.node, getCurrentTask(te.getRobotID()));
		canReplan.put(te.getRobotID(), new AtomicBoolean(true));
		return tet;
	}
	
	public VEHICLE_STATE getVehicleState(int robotID) {
		if (!(trackers.get(robotID) instanceof TrajectoryEnvelopeTrackerROS)) return VEHICLE_STATE._IGNORE_;
		return ((TrajectoryEnvelopeTrackerROS)trackers.get(robotID)).getVehicleState();
	}
		
	public void replacePath(int robotID, PoseSteering[] newPath, int breakingPathIndex, Set<Integer> lockedRobotIDs) {
		synchronized(this.solver) {
			if (breakingPathIndex > newPath.length-1) {
				metaCSPLogger.warning("Invalid braking path index. We will not replace the path (new path length : " + newPath.length + ", braking path index: " + breakingPathIndex + ")!");
				return;
			}
			orunav_msgs.Task currentTask = currentTasks.get(robotID);
			orunav_msgs.Path pathROS = node.getTopicMessageFactory().newFromType(orunav_msgs.Path._TYPE);
			for (int i = 0; i < newPath.length; i++) {
				PoseSteering pose = newPath[i];
				orunav_msgs.PoseSteering poseROS = node.getTopicMessageFactory().newFromType(orunav_msgs.PoseSteering._TYPE);
				poseROS.getPose().getPosition().setX(pose.getX());
				poseROS.getPose().getPosition().setY(pose.getY());
				poseROS.getPose().getPosition().setZ(pose.getZ());
				Quaternion quat = new Quaternion(pose.getTheta());
				poseROS.getPose().getOrientation().setW(quat.getW());
				poseROS.getPose().getOrientation().setX(quat.getX());
				poseROS.getPose().getOrientation().setY(quat.getY());
				poseROS.getPose().getOrientation().setZ(quat.getZ());
				poseROS.setSteering(pose.getSteering());
				pathROS.getPath().add(poseROS);
				if (i == 0) {
					pathROS.setTargetStart(poseROS);
					currentTask.getTarget().setStart(poseROS);
				}
				if (i == newPath.length-1) {
					pathROS.setTargetGoal(poseROS);
					currentTask.getTarget().setGoal(poseROS);
				}
			}
			currentTask.setPath(pathROS);
			setCurrentTask(robotID, currentTask);
			super.replacePath(robotID, newPath, breakingPathIndex, lockedRobotIDs);
		}
	}
	
	/**
	 * Truncate the {@link TrajectoryEnvelope} of a given robot at the closest dynamically-feasible path point. This path point is computed via the robot's {@link ForwardModel}.
	 * @param robotID The ID of the robot whose {@link TrajectoryEnvelope} should be truncated.
	 * @param pathIndex The path index at which the envelope should be truncated (ATTENTION: it should be retrieved after calling the brake_task service).
	 * @return <code>true</code> iff the envelope is successfully truncated at the desired path index.
	 */
	@Override
	public boolean truncateEnvelope(final int robotID) {
		synchronized(solver) {
			metaCSPLogger.info("Locking the solver to abort mission of robot " + robotID + ".");
		
			AbstractTrajectoryEnvelopeTracker tet = null;
			synchronized(trackers) {
				metaCSPLogger.info("Locking the trackers to abort mission of robot " + robotID + ".");
				tet = trackers.get(robotID); 
			}
			if (tet instanceof TrajectoryEnvelopeTrackerDummy) {
				metaCSPLogger.info("Cannot truncate envelope of robot " + robotID + " since the robot is parked.");
				return false;
			}
			final TrajectoryEnvelope te = tet.getTrajectoryEnvelope();
					
			if (!canReplan.get(robotID).compareAndSet(true, false)) {
					metaCSPLogger.info("Cannot truncate envelope of robot " + robotID + " since already re-planning.");
					return false;
			}

			
			//Make the robot braking
			ServiceClient<BrakeTaskRequest, BrakeTaskResponse> serviceClient = null;
			try {
				System.out.println("-------> Going to call service client: /robot" + te.getRobotID() + "/brake_task");
				serviceClient = node.newServiceClient("/robot" + te.getRobotID() + "/brake_task", BrakeTask._TYPE);
			}
			catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
			final BrakeTaskRequest request = serviceClient.newMessage();
			final ArrayList<Boolean> ret = new ArrayList<Boolean>();
			serviceClient.call(request, new ServiceResponseListener<BrakeTaskResponse>() {
				@Override
				public void onSuccess(BrakeTaskResponse response) {
					metaCSPLogger.info("Braking envelope of Robot" + robotID + " at " + response.getCurrentPathIdx() + ".");
					currentBreakingPathIndex.put(robotID, response.getCurrentPathIdx());
					try { Thread.sleep(1000); } catch (Exception e) {}; //Let the controller change the status
					ret.add(new Boolean(true));
				}
				@Override
				public void onFailure(RemoteException arg0) {
					System.out.println("Failed to brake service of robot " + robotID);
					ret.add(new Boolean(false));
				}
			});		
			while (ret.isEmpty()) {
				try { Thread.sleep(100); } catch (InterruptedException e) {}
			}
			if (ret.get(0)) {
				synchronized (solver) {
					//replace the path of this robot (will compute new envelope)
					int breakingIndex = Math.max(currentBreakingPathIndex.get(robotID), getRobotReport(robotID).getPathIndex());
					PoseSteering[] truncatedPath = Arrays.copyOf(te.getTrajectory().getPoseSteering(), breakingIndex+1);
					replacePath(robotID, truncatedPath, truncatedPath.length-1, new HashSet<Integer>(robotID));
					metaCSPLogger.info("Truncating " + te + " at " + breakingIndex + ".");	
				}
			}
			canReplan.get(robotID).set(true);

			return true;
		}
	}
	
	/**
	 * Re-plan the path for a given robot.
	 * @param robotID The robot which path should be re-planned.
	 * @return <code>true</code> if re-planning is correctly spawned.
	 */
	public boolean replanEnvelope(final int robotID) {
			
		synchronized(solver) {
			metaCSPLogger.info("Locking the solver to replan for robot " + robotID + ".");
			
			//Return false if the robot is dummy or if it is already planning
			AbstractTrajectoryEnvelopeTracker tracker = null;
			synchronized(trackers) {
				metaCSPLogger.info("Locking the tracker to replan for robot " + robotID + ".");
				tracker = trackers.get(robotID);
			}
			if (tracker instanceof TrajectoryEnvelopeTrackerDummy) return false;
			
			orunav_msgs.Task currentTask = getCurrentTask(robotID);
			if (currentTask == null) {
				metaCSPLogger.info("Invalid task for robot " + robotID + ".");
				return false;
			}

			//There is another re-planning instance already ongoing.
			if (!canReplan.get(robotID).compareAndSet(true, false)) {
				metaCSPLogger.info("Cannot replan envelope of robot " + robotID + " since already re-planning.");
				return false;
			}
			
			final int taskID = currentTask.getTarget().getTaskId();
			orunav_msgs.ReplanStatus msg = node.getTopicMessageFactory().newFromType(orunav_msgs.ReplanStatus._TYPE);
			msg.setRobotId(robotID);
			msg.setTaskId(taskID);
			msg.setStatus(orunav_msgs.ReplanStatus.REPLAN_SERVICE_START);
			replanStatusPublisher.publish(msg);
			
			final ArrayList<Boolean> isBraked = new ArrayList<Boolean>();
			if (((TrajectoryEnvelopeTrackerROS)tracker).isBraking() != null && ((TrajectoryEnvelopeTrackerROS)tracker).isBraking().booleanValue()) {
				metaCSPLogger.info("Service brake already called for Robot" + robotID + ".");
				isBraked.add(new Boolean(true));
			}
			else {	
				//Make the robot braking
				ServiceClient<BrakeTaskRequest, BrakeTaskResponse> serviceClient = null;
				try {
					System.out.println("-------> Going to call service client: /robot" + robotID + "/brake_task");
					serviceClient = node.newServiceClient("/robot" + robotID + "/brake_task", BrakeTask._TYPE);
				}
				catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
				final BrakeTaskRequest request = serviceClient.newMessage();
				serviceClient.call(request, new ServiceResponseListener<BrakeTaskResponse>() {
					@Override
					public void onSuccess(BrakeTaskResponse response) {
						currentBreakingPathIndex.put(robotID, response.getCurrentPathIdx());
						metaCSPLogger.info("Braking envelope of Robot" + robotID + " at " + response.getCurrentPathIdx() + ".");
						try { Thread.sleep(1000); } catch (Exception e) {}; //Let the controller change the status
						isBraked.add(new Boolean(true));
					}
					@Override
					public void onFailure(RemoteException arg0) {
						System.out.println("Failed to brake service of robot " + robotID);
						isBraked.add(new Boolean(false));
					}
				});
				while (isBraked.isEmpty()) {
					try { Thread.sleep(100); } catch (InterruptedException e) {}
				}
			}
				
			//Spawn a re-plan thread if the robot has braked
			if (isBraked.get(0)) {
				//get all the robots in the same weakly connected component (we need to place obstacle)
				SimpleDirectedGraph<Integer,Dependency> g = depsToGraph(currentDependencies);
				ConnectivityInspector<Integer,Dependency> connInsp = new ConnectivityInspector<Integer,Dependency>(g);
				final Set<Integer> allConnectedRobots = g.containsVertex(robotID) ? (HashSet<Integer>) connInsp.connectedSetOf(robotID) : new HashSet<Integer>(); //it may be empty if the robot has no dependency
				final Set<Integer> robotsToReplan = new HashSet<Integer>();
				robotsToReplan.add(robotID); 
				final TrajectoryEnvelope te = tracker.getTrajectoryEnvelope();
				Dependency dep = new Dependency(te, null, Math.max(currentBreakingPathIndex.get(robotID), getRobotReport(robotID).getPathIndex()), 0);
				//Create a virtual dependency to keep track of the robot start and goal till the replanning will ends.
				System.out.println("Stopping point: " + dep.toString());
				synchronized(replanningStoppingPoints) {
					replanningStoppingPoints.put(robotID, dep);
				}
				metaCSPLogger.info("Will re-plan for robot " + robotID + " (" + allConnectedRobots + ")...");
				new Thread() {
					public void run() {
						boolean ret = rePlanPath(robotsToReplan, allConnectedRobots);
						orunav_msgs.ReplanStatus msg = node.getTopicMessageFactory().newFromType(orunav_msgs.ReplanStatus._TYPE);
						msg.setRobotId(robotID);
						msg.setTaskId(taskID);
						msg.setStatus(ret ? orunav_msgs.ReplanStatus.REPLAN_SUCCESS : orunav_msgs.ReplanStatus.REPLAN_FAILURE);
						replanStatusPublisher.publish(msg);
						canReplan.get(robotID).set(true);
					}
				}.start();
			}
				
		}
		return true;
	}
}
