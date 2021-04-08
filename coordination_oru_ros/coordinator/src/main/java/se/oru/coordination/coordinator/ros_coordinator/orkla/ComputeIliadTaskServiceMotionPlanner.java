package se.oru.coordination.coordinator.ros_coordinator.orkla;

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

import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.motionplanning.OccupancyMap;
import se.oru.coordination.coordinator.ros_coordinator.IliadItem;
import se.oru.coordination.coordinator.ros_coordinator.TrajectoryEnvelopeCoordinatorROS;
import se.oru.coordination.coordinator.ros_coordinator.IliadMission.OPERATION_TYPE;
import se.oru.coordination.coordinator.ros_coordinator.TrajectoryEnvelopeTrackerROS.VEHICLE_STATE;

import geometry_msgs.Point;
import orunav_msgs.Task;
import orunav_msgs.ComputeTask;
import orunav_msgs.ComputeTaskRequest;
import orunav_msgs.ComputeTaskResponse;
import orunav_msgs.Operation;
import orunav_msgs.Shape;
import orunav_msgs.RobotTarget;

public class ComputeIliadTaskServiceMotionPlanner extends AbstractMotionPlanner {
	private ConnectedNode node = null;
	private HashMap<Integer,Integer> robotIDstoGoalIDs = new HashMap<Integer,Integer>();
	private TrajectoryEnvelopeCoordinatorROS tec = null;
	private boolean computing = false;
	private boolean outcome = false;
	private int robotID = -1;
	private OPERATION_TYPE operationType = null;
	private IliadItem[] pickItems = null;
	private boolean ignorePickItems = false;
	private boolean copyGoalOperationToStartoperation = false;
	
	public ComputeIliadTaskServiceMotionPlanner(int robotID, ConnectedNode node, TrajectoryEnvelopeCoordinatorROS tec) {
		this.node = node;
		this.tec = tec;
		this.robotID = robotID;
	}
	
	@Override
	public AbstractMotionPlanner getCopy(boolean copyObstacles) {
		ComputeIliadTaskServiceMotionPlanner ret = new ComputeIliadTaskServiceMotionPlanner(this.robotID, this.node, this.tec);
		if (this.om != null) ret.om = new OccupancyMap(this.om, copyObstacles);
		ret.setOperationType(this.operationType);
		ret.setPickItems(this.pickItems);
		return ret;
	}
	
	public void setOperationType(OPERATION_TYPE operationType) {
		this.operationType = operationType;
	}
	
	public void setPickItems(IliadItem[] items) {
		this.pickItems = items;
	}
	
	public void setCopyGoalOperationToStartoperation(boolean copyGoalOperationToStartoperation) {
		this.copyGoalOperationToStartoperation = copyGoalOperationToStartoperation;
	}
	
	public void setIgnorePickItems(boolean ignorePickItems) {
		this.ignorePickItems = ignorePickItems;
	}
	
	private void callComputeTaskService(boolean startFromCurrentState) {

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
			point1.setX(this.goal[0].getX());
			point1.setY(this.goal[0].getY());
			point1.setZ(0.0);
			Quaternion gQuat1 = new Quaternion(this.goal[0].getTheta());
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
				point2.setX(this.start.getX());
				point2.setY(this.start.getY());
				point2.setZ(0.0);
				Quaternion gQuat2 = new Quaternion(this.start.getTheta());
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
			
			//Add extra obstacles
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
	
					ArrayList<PoseSteering> path = new ArrayList<PoseSteering>();
					for (int i = 0; i < arg0.getTask().getPath().getPath().size(); i++) {
						orunav_msgs.PoseSteering onePS = arg0.getTask().getPath().getPath().get(i);
						Quaternion quat = new Quaternion(onePS.getPose().getOrientation().getX(), onePS.getPose().getOrientation().getY(), onePS.getPose().getOrientation().getZ(), onePS.getPose().getOrientation().getW());
						PoseSteering ps = new PoseSteering(onePS.getPose().getPosition().getX(), onePS.getPose().getPosition().getY(), quat.getTheta(), onePS.getSteering());
						path.add(ps);
					}
					pathPS = path.toArray(new PoseSteering[path.size()]);
					
					//Operations used by the current execution service
					Operation goalOp = node.getTopicMessageFactory().newFromType(Operation._TYPE);
					goalOp.setOperation(operationType.ordinal());
					//goalOp.setOperation(Operation.NO_OPERATION);
					if (operationType.equals(OPERATION_TYPE.PICK_ITEMS)) {
						if (ignorePickItems) {
							System.out.println("Ignoring PICK_ITEMS operation (see launch file)");
							goalOp.setOperation(Operation.NO_OPERATION);
						}
						orunav_msgs.IliadItemArray iliadItemArrayMsg = node.getTopicMessageFactory().newFromType(orunav_msgs.IliadItemArray._TYPE);
						ArrayList<orunav_msgs.IliadItem> itemList = new ArrayList<orunav_msgs.IliadItem>();
						for (IliadItem item : pickItems) {
							orunav_msgs.IliadItem iliadItemMsg = node.getTopicMessageFactory().newFromType(orunav_msgs.IliadItem._TYPE);
							iliadItemMsg.setName(item.getName());
							geometry_msgs.Point point = node.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
							point.setX(item.getX());
							point.setY(item.getY());
							point.setZ(item.getZ());
							iliadItemMsg.setPosition(point);
							iliadItemMsg.setRotationType(item.getRotationType().ordinal());
							itemList.add(iliadItemMsg);
						}
						iliadItemArrayMsg.setItems(itemList);
						goalOp.setItemlist(iliadItemArrayMsg);
					}
					arg0.getTask().getTarget().setGoalOp(goalOp);

					Operation startOp = node.getTopicMessageFactory().newFromType(Operation._TYPE);
					startOp.setOperation(Operation.NO_OPERATION);
					if (copyGoalOperationToStartoperation) arg0.getTask().getTarget().setStartOp(goalOp);
					else arg0.getTask().getTarget().setStartOp(startOp);
					
					tec.setCurrentTask(arg0.getTask().getTarget().getRobotId(), arg0.getTask()); //FIXME This is a logical error
					computing = false;
				}
			});
		}
	}
		
	@Override
	public boolean doPlanning() {
		//Start from the current position only if the robot is idle. 
		boolean startFromCurrentState = true;
		if (!(tec.getVehicleState(robotID).equals(VEHICLE_STATE.WAITING_FOR_TASK) ||
				tec.getVehicleState(robotID).equals(VEHICLE_STATE._IGNORE_))) {
			//System.out.println("Not planning because Robot" + robotID + " is not idle (in state " + tec.getVehicleState(robotID) + ")");
			startFromCurrentState = false;
		}
		
		this.callComputeTaskService(startFromCurrentState);
		while (computing) try { Thread.sleep(100); } catch (InterruptedException e) { e.printStackTrace(); }
		return outcome;
	}
}
