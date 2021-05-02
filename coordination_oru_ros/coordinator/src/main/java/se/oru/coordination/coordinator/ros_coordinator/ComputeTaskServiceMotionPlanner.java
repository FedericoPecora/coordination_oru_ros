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
import se.oru.coordination.coordination_oru.motionplanning.OccupancyMap;
import se.oru.coordination.coordinator.ros_coordinator.TrajectoryEnvelopeTrackerROS.VEHICLE_STATE;
import se.oru.coordination.coordinator.ros_coordinator.IliadMission.OPERATION_TYPE;

public class ComputeTaskServiceMotionPlanner extends AbstractMotionPlanner {
	
	private ConnectedNode node = null;
	private HashMap<Integer,Integer> robotIDstoGoalIDs = new HashMap<Integer,Integer>();
	private TrajectoryEnvelopeCoordinatorROS tec = null;
	private boolean computing = false;
	private boolean outcome = false;
	private int robotID = -1;
	private boolean startFromCurrentState = true;
	
	public ComputeTaskServiceMotionPlanner(int robotID, ConnectedNode node, TrajectoryEnvelopeCoordinatorROS tec) {
		this.node = node;
		this.tec = tec;
		this.robotID = robotID;
	}
	
	@Override
	public AbstractMotionPlanner getCopy(boolean copyObstacles) {
		ComputeTaskServiceMotionPlanner ret = new ComputeTaskServiceMotionPlanner(this.robotID, this.node, this.tec);
		if (this.om != null) ret.om = new OccupancyMap(this.om, copyObstacles);
		return ret;
	}
	
	public void setStartFromCurrentState(boolean startFromCurrentState) {
		this.startFromCurrentState = startFromCurrentState;
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
			request.setStartFromCurrentState(startFromCurrentState);
			RobotTarget rt = node.getTopicMessageFactory().newFromType(RobotTarget._TYPE);
			orunav_msgs.PoseSteering ps1 = node.getTopicMessageFactory().newFromType(orunav_msgs.PoseSteering._TYPE);
			geometry_msgs.Pose gpose1 = node.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
			geometry_msgs.Point point1 = node.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
			geometry_msgs.Quaternion quat1 = node.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);
			point1.setX(goalPose.getX());
			point1.setY(goalPose.getY());
			point1.setZ(0.0);
			Quaternion gQuat1 = new Quaternion(goalPose.getTheta());
			quat1.setW(gQuat1.getW());
			quat1.setX(gQuat1.getX());
			quat1.setY(gQuat1.getY());
			quat1.setZ(gQuat1.getZ());
			gpose1.setPosition(point1);
			gpose1.setOrientation(quat1);
			ps1.setPose(gpose1);
			ps1.setSteering(0.0);
			rt.setGoal(ps1);
			if (!startFromCurrentState) {
				orunav_msgs.PoseSteering ps2 = node.getTopicMessageFactory().newFromType(orunav_msgs.PoseSteering._TYPE);
				geometry_msgs.Pose gpose2 = node.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
				geometry_msgs.Point point2 = node.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
				geometry_msgs.Quaternion quat2 = node.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);
				point2.setX(start.getX());
				point2.setY(start.getY());
				point2.setZ(0.0);
				Quaternion gQuat2 = new Quaternion(start.getTheta());
				quat2.setW(gQuat2.getW());
				quat2.setX(gQuat2.getX());
				quat2.setY(gQuat2.getY());
				quat2.setZ(gQuat2.getZ());
				gpose2.setPosition(point2);
				gpose2.setOrientation(quat2);
				ps2.setPose(gpose2);
				ps2.setSteering(0.0);
				rt.setStart(ps2);
			}
			rt.setRobotId(robotID);
			rt.setTaskId(goalID);
			rt.setGoalId(goalID);
			request.setTarget(rt);
			
			if (this.om != null) {
				for (Geometry obs : this.om.getObstacles()) {
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
					
					//Operations used by the current execution service
					arg0.getTask().getTarget().getStartOp().setOperation(OPERATION_TYPE.NO_OPERATION.ordinal());
					arg0.getTask().getTarget().getGoalOp().setOperation(OPERATION_TYPE.NO_OPERATION.ordinal());					
					tec.setCurrentTask(arg0.getTask().getTarget().getRobotId(), arg0.getTask()); //FIXME This is a logical mistake.
	
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
		//Start from the current position only if the robot is idle. 
		if (!(tec.getVehicleState(robotID).equals(VEHICLE_STATE.WAITING_FOR_TASK) ||
				tec.getVehicleState(robotID).equals(VEHICLE_STATE._IGNORE_))) {
			//System.out.println("Not planning because Robot" + robotID + " is not idle (in state " + tec.getVehicleState(robotID) + ")");
			startFromCurrentState = false;
		}
		
		this.callComputeTaskService(this.goal[0], robotID);
		while (computing) try { Thread.sleep(100); } catch (InterruptedException e) { e.printStackTrace(); }
		return outcome;
	}
}
