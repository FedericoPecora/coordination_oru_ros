package se.oru.coordination.coordinator.ros_coordinator;

import java.util.HashMap;
import java.util.concurrent.TimeUnit;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.ros.exception.ServiceException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceResponseBuilder;

import com.vividsolutions.jts.geom.Geometry;

import orunav_msgs.Task;
import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.TrackingCallback;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;

public class TrajectoryEnvelopeCoordinatorROS extends TrajectoryEnvelopeCoordinator {

	protected ConnectedNode node = null;
	protected HashMap<Integer,Task> currentTasks = new HashMap<Integer,Task>();
	public static enum VEHICLE_STATE {WAITING_FOR_TASK, PERFORMING_START_OPERATION, DRIVING, PERFORMING_GOAL_OPERATION, TASK_FAILED, WAITING_FOR_TASK_INTERNAL, DRIVING_SLOWDOWN, AT_CRITICAL_POINT}

	private void setupAbortService() {
		node.newServiceServer("coordinator/abort", orunav_msgs.Abort._TYPE, new ServiceResponseBuilder<orunav_msgs.AbortRequest, orunav_msgs.AbortResponse>() {
			@Override
			public void build(orunav_msgs.AbortRequest arg0, orunav_msgs.AbortResponse arg1) throws ServiceException {
				System.out.println(">>>>>>>>>>>>>> ABORTING Robot" + arg0.getRobotID());
				truncateEnvelope(arg0.getRobotID());
			}
		});
	}
	
	private void setupReverseService() {
		node.newServiceServer("coordinator/reverse", orunav_msgs.Abort._TYPE, new ServiceResponseBuilder<orunav_msgs.AbortRequest, orunav_msgs.AbortResponse>() {
			@Override
			public void build(orunav_msgs.AbortRequest arg0, orunav_msgs.AbortResponse arg1) throws ServiceException {
				System.out.println(">>>>>>>>>>>>>> REVERSING Robot" + arg0.getRobotID());
				reverseEnvelope(arg0.getRobotID());
			}
		});
	}

	public TrajectoryEnvelopeCoordinatorROS(int CONTROL_PERIOD, double TEMPORAL_RESOLUTION, final ConnectedNode connectedNode) {
		super(CONTROL_PERIOD, TEMPORAL_RESOLUTION);
		this.node = connectedNode;
		setupAbortService();
		setupReverseService();
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

	public Task getCurrentTask(int robotID) {
		return this.currentTasks.get(robotID);
	}

	@Override
	public AbstractTrajectoryEnvelopeTracker getNewTracker(TrajectoryEnvelope te, TrackingCallback cb) {
		TrajectoryEnvelopeTrackerROS tet = new TrajectoryEnvelopeTrackerROS(te, this.TEMPORAL_RESOLUTION, this, cb, this.node, getCurrentTask(te.getRobotID()));
		return tet;
	}

	//Override necessary in next release... 
	@Override
	protected PoseSteering[] doReplanning(Pose fromPose, Pose toPose, Geometry... obstaclesToConsider) {
		// TODO Auto-generated method stub
		return null;
	}

}
