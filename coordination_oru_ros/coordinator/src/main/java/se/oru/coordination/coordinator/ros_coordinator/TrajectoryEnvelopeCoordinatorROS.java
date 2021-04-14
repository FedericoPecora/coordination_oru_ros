package se.oru.coordination.coordinator.ros_coordinator;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.TimeUnit;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Quaternion;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.ros.exception.ServiceException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceResponseBuilder;

import com.vividsolutions.jts.geom.Geometry;

import orunav_msgs.Task;
import orunav_msgs.Path;
import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.TrackingCallback;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
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
			super.replacePath(robotID, newPath, breakingPathIndex, lockedRobotIDs);
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
		}
	}

}
