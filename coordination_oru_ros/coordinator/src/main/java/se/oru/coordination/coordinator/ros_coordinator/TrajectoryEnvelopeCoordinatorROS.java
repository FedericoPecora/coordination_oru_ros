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
//import orunav_msgs.IsFree; //FIXME: commit the new message
import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.TrackingCallback;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordinator.ros_coordinator.TrajectoryEnvelopeTrackerROS.VEHICLE_STATE;

public class TrajectoryEnvelopeCoordinatorROS extends TrajectoryEnvelopeCoordinator {

	protected ConnectedNode node = null;
	protected HashMap<Integer,Task> currentTasks = new HashMap<Integer,Task>();

	public TrajectoryEnvelopeTrackerROS getCurrentTracker(int robotID) {
		return (TrajectoryEnvelopeTrackerROS)this.trackers.get(robotID);
	}
	
	private void setupAbortService() {
		node.newServiceServer("coordinator/abort", orunav_msgs.Abort._TYPE, new ServiceResponseBuilder<orunav_msgs.AbortRequest, orunav_msgs.AbortResponse>() {
			@Override
			public void build(orunav_msgs.AbortRequest arg0, orunav_msgs.AbortResponse arg1) throws ServiceException {
				System.out.println(">>>>>>>>>>>>>> ABORTING Robot" + arg0.getRobotID());
				int cp = truncateEnvelope(arg0.getRobotID(), !arg0.getForce(), 3); //Threshold set according to https://github.com/OrebroUniversity/navigation_oru-release/blob/master/orunav_vehicle_execution/include/orunav_vehicle_execution/trajectory_generation.h
				if (cp != -2) {
					//getCurrentTracker(arg0.getRobotID()).setCriticalPoint(cp);
					arg1.setSuccess(true);
					return;
				}
				arg1.setSuccess(false);
				arg1.setMessage("Robot" + arg0.getRobotID() + " is planning. The mission cannot be aborted now.");
			}
		});
	}
	
	@Deprecated
	private void setupReverseService() {
		node.newServiceServer("coordinator/reverse", orunav_msgs.Abort._TYPE, new ServiceResponseBuilder<orunav_msgs.AbortRequest, orunav_msgs.AbortResponse>() {
			@Override
			public void build(orunav_msgs.AbortRequest arg0, orunav_msgs.AbortResponse arg1) throws ServiceException {
				System.out.println(">>>>>>>>>>>>>> REVERSING Robot" + arg0.getRobotID());
				int cp = reverseEnvelope(arg0.getRobotID());
				if (cp != -2) {
					getCurrentTracker(arg0.getRobotID()).setCriticalPoint(cp);
					arg1.setSuccess(true);
					return;
				}
				arg1.setSuccess(false);
			}
		});
	}

	/*private void setupIsFreeService() {
		node.newServiceServer("coordinator/is_free", orunav_msgs.IsFree._TYPE, new ServiceIsFreeBuilder<orunav_msgs.IsFreeRequest, orunav_msgs.IsFreeResponse>() {
			@Override
			public void build(orunav_msgs.IsFreeRequest arg0, orunav_msgs.AbortResponse arg1) throws ServiceException {
				arg1.success = this.isFree(arg0.getRobotID());
				System.out.println(">>>>>>>>>>>>>> IS Robot" + arg0.getRobotID() + " FREE? " + arg1.success);
			}
		});
	}*/
	
	public TrajectoryEnvelopeCoordinatorROS(int CONTROL_PERIOD, double TEMPORAL_RESOLUTION, final ConnectedNode connectedNode) {
		super(CONTROL_PERIOD, TEMPORAL_RESOLUTION);
		this.node = connectedNode;
		setupAbortService();
		setupReverseService();
		//setupIsFreeService();
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
	
	public VEHICLE_STATE getVehicleState(int robotID) {
		if (!(trackers.get(robotID) instanceof TrajectoryEnvelopeTrackerROS)) return VEHICLE_STATE._IGNORE_;
		return ((TrajectoryEnvelopeTrackerROS)trackers.get(robotID)).getVehicleState();
	}


}
