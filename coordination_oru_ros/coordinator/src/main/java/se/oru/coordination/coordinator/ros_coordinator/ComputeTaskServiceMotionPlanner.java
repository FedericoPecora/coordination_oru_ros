package se.oru.coordination.coordinator.ros_coordinator;

import java.util.ArrayList;
import java.util.HashMap;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Quaternion;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

import geometry_msgs.Point;
import orunav_msgs.ComputeTask;
import orunav_msgs.ComputeTaskRequest;
import orunav_msgs.ComputeTaskResponse;
import orunav_msgs.RobotTarget;
import orunav_msgs.Shape;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordinator.ros_coordinator.TrajectoryEnvelopeTrackerROS.VEHICLE_STATE;

public class ComputeTaskServiceMotionPlanner extends AbstractMotionPlanner {
	
	private ConnectedNode node = null;
	private HashMap<Integer,Integer> robotIDstoGoalIDs = new HashMap<Integer,Integer>();
	private TrajectoryEnvelopeCoordinatorROS tec = null;
	private boolean computing = false;
	private boolean outcome = false;
	private int robotID = -1;

	public ComputeTaskServiceMotionPlanner(int robotID, ConnectedNode node, TrajectoryEnvelopeCoordinatorROS tec) {
		this.node = node;
		this.tec = tec;
		this.robotID = robotID;
	}
	
	private void callComputeTaskService(Pose goalPose, final int robotID) {

		if (!computing) {
			computing = true;
	
			ServiceClient<ComputeTaskRequest, ComputeTaskResponse> serviceClient;
			try { serviceClient = node.newServiceClient("/robot" + robotID + "/compute_task", ComputeTask._TYPE); }
			catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
	
			if (!robotIDstoGoalIDs.keySet().contains(robotID)) robotIDstoGoalIDs.put(robotID, 1);
			else robotIDstoGoalIDs.put(robotID, robotIDstoGoalIDs.get(robotID)+1);
			final int goalID = robotIDstoGoalIDs.get(robotID);
	
			final ComputeTaskRequest request = serviceClient.newMessage();
			request.setStartFromCurrentState(true);
			RobotTarget rt = node.getTopicMessageFactory().newFromType(RobotTarget._TYPE);
			orunav_msgs.PoseSteering ps = node.getTopicMessageFactory().newFromType(orunav_msgs.PoseSteering._TYPE);
			geometry_msgs.Pose gpose = node.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
			geometry_msgs.Point point = node.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
			geometry_msgs.Quaternion quat = node.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);
			point.setX(goalPose.getX());
			point.setY(goalPose.getY());
			point.setZ(0.0);
			Quaternion gQuat = new Quaternion(goalPose.getTheta());
			quat.setW(gQuat.getW());
			quat.setX(gQuat.getX());
			quat.setY(gQuat.getY());
			quat.setZ(gQuat.getZ());
			gpose.setPosition(point);
			gpose.setOrientation(quat);
			ps.setPose(gpose);
			ps.setSteering(0.0);
			rt.setGoal(ps);
			rt.setRobotId(robotID);
			rt.setTaskId(goalID);
			rt.setGoalId(goalID);
			request.setTarget(rt);
			
			for (Geometry obs : this.om.obstacles) {
				Shape shape = node.getTopicMessageFactory().newFromType(Shape._TYPE);
				for (Coordinate coord : obs.getCoordinates()) {
					Point pnt = node.getTopicMessageFactory().newFromType(Point._TYPE);
					pnt.setX(coord.x);
					pnt.setY(coord.y);
					shape.getPoints().add(pnt);
				}
				//Shape is a polygon (type = 1)
				shape.setType(1);
				request.getExtraObstacles().add(shape);
				System.out.println("Added extra obstacle when planning for Robot" + robotID);
			}
	
			serviceClient.call(request, new ServiceResponseListener<ComputeTaskResponse>() {
	
				@Override
				public void onFailure(RemoteException arg0) {
					System.out.println("FAILED to call ComputeTask service for Robot" + robotID + " (goalID: " + goalID + ")");
					outcome = false;
					computing = false;
				}
	
				@Override
				public void onSuccess(ComputeTaskResponse arg0) {
					System.out.println("Successfully called ComputeTask service for Robot" + robotID + " (goalID: " + goalID + ")");
					outcome = true;
					
					tec.setCurrentTask(arg0.getTask().getTarget().getRobotId(), arg0.getTask());
	
					ArrayList<PoseSteering> path = new ArrayList<PoseSteering>();
					for (int i = 0; i < arg0.getTask().getPath().getPath().size(); i++) {
						orunav_msgs.PoseSteering onePS = arg0.getTask().getPath().getPath().get(i);
						Quaternion quat = new Quaternion(onePS.getPose().getOrientation().getX(), onePS.getPose().getOrientation().getY(), onePS.getPose().getOrientation().getZ(), onePS.getPose().getOrientation().getW());
						PoseSteering ps = new PoseSteering(onePS.getPose().getPosition().getX(), onePS.getPose().getPosition().getY(), quat.getTheta(), onePS.getSteering());
						path.add(ps);
					}
					pathPS = path.toArray(new PoseSteering[path.size()]);
					computing = false;
				}
			});
		}
	}
	
	@Override
	public boolean doPlanning() {
		//TODO: refuse to do planning if the robot is not idle,
		//		as compute_task service will use current pose of
		//		robot as initial pose. 
		if (!(
				tec.getVehicleState(robotID).equals(VEHICLE_STATE.AT_CRITICAL_POINT) ||
				tec.getVehicleState(robotID).equals(VEHICLE_STATE.WAITING_FOR_TASK) ||
				tec.getVehicleState(robotID).equals(VEHICLE_STATE._IGNORE_))) {
			System.out.println("Not planning because Robot" + robotID + " is not idle (in state " + tec.getVehicleState(robotID) + ")");
			return false;
		}
		
		this.callComputeTaskService(this.goal[0], robotID);
		while (computing) try { Thread.sleep(100); } catch (InterruptedException e) { e.printStackTrace(); }
		return outcome;
	}

}
