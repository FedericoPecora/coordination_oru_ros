package se.oru.coordination.coordinator.ros_coordinator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Quaternion;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceResponseListener;

import com.vividsolutions.jts.geom.Geometry;

import orunav_msgs.Task;
import orunav_msgs.BrakeTask;
import orunav_msgs.BrakeTaskRequest;
import orunav_msgs.BrakeTaskResponse;
import orunav_msgs.Path;
import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.TrackingCallback;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeTrackerDummy;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordinator.ros_coordinator.IliadMission.OPERATION_TYPE;
import se.oru.coordination.coordinator.ros_coordinator.TrajectoryEnvelopeTrackerROS.VEHICLE_STATE;

public class TrajectoryEnvelopeCoordinatorROS extends TrajectoryEnvelopeCoordinator {

	protected ConnectedNode node = null;
	protected HashMap<Integer, orunav_msgs.Task> currentTasks = new HashMap<Integer, orunav_msgs.Task>();

	public TrajectoryEnvelopeTrackerROS getCurrentTracker(int robotID) {
		return (TrajectoryEnvelopeTrackerROS)this.trackers.get(robotID);
	}
		
	public TrajectoryEnvelopeCoordinatorROS(int CONTROL_PERIOD, double TEMPORAL_RESOLUTION, final ConnectedNode connectedNode) {
		super(CONTROL_PERIOD, TEMPORAL_RESOLUTION);
		this.node = connectedNode;
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
		return tet;
	}
	
	public VEHICLE_STATE getVehicleState(int robotID) {
		if (!(trackers.get(robotID) instanceof TrajectoryEnvelopeTrackerROS)) return VEHICLE_STATE._IGNORE_;
		return ((TrajectoryEnvelopeTrackerROS)trackers.get(robotID)).getVehicleState();
	}
		
	public void replacePath(int robotID, PoseSteering[] newPath, int breakingPathIndex, Set<Integer> lockedRobotIDs) {
		synchronized(this.solver) {
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
		ServiceClient<BrakeTaskRequest, BrakeTaskResponse> serviceClient;
		try {
			System.out.println("-------> Going to call service: /robot" + robotID + "/brake_task");
			serviceClient = node.newServiceClient("/robot" + robotID + "/brake_task", BrakeTask._TYPE);
		}
		catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
		final BrakeTaskRequest request = serviceClient.newMessage();
		serviceClient.call(request, new ServiceResponseListener<BrakeTaskResponse>() {
			@Override
			public void onSuccess(BrakeTaskResponse response) {
				metaCSPLogger.info("Braking envelope of Robot" + robotID + " at " + response.getCurrentPathIdx() + ".");
				try { Thread.sleep(1000); } catch (Exception e) {}; //Let the controller change the status
				
				synchronized (solver) {
					synchronized (replanningStoppingPoints) {
						if (replanningStoppingPoints.containsKey(robotID)) {
							metaCSPLogger.info("Cannot truncate envelope of robot " + robotID + " at " + response.getCurrentPathIdx() + ".");
						}
					}
					TrajectoryEnvelope te = getCurrentTrajectoryEnvelope(robotID);
					AbstractTrajectoryEnvelopeTracker tet = null;
					synchronized(trackers) {
						tet = trackers.get(robotID); 
					}
					if (!(tet instanceof TrajectoryEnvelopeTrackerDummy)) {		
						//replace the path of this robot (will compute new envelope)
						PoseSteering[] truncatedPath = Arrays.copyOf(te.getTrajectory().getPoseSteering(),response.getCurrentPathIdx()+1);
						replacePath(robotID, truncatedPath, truncatedPath.length-1, new HashSet<Integer>(robotID));
						metaCSPLogger.info("Truncating " + te + " at " + response.getCurrentPathIdx() + ".");
					}
				}
			}
			@Override
			public void onFailure(RemoteException arg0) {
				System.out.println("Failed to brake service of robot " + robotID);
			}
		});
		return true;
		
	}

}
