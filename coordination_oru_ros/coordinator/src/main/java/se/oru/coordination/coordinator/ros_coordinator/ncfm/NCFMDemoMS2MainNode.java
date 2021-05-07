package se.oru.coordination.coordinator.ros_coordinator.ncfm;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.TimeUnit;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Quaternion;
import org.ros.concurrent.CancellableLoop;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Subscriber;

import com.vividsolutions.jts.geom.Coordinate;

import orunav_msgs.ComputeTask;
import orunav_msgs.ComputeTaskRequest;
import orunav_msgs.ComputeTaskResponse;
import orunav_msgs.Operation;
import orunav_msgs.RobotTarget;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.util.RVizVisualization;
import se.oru.coordination.coordinator.ros_coordinator.IliadItem;
import se.oru.coordination.coordinator.ros_coordinator.IliadMission;
import se.oru.coordination.coordinator.ros_coordinator.IliadMission.OPERATION_TYPE;
import se.oru.coordination.coordinator.ros_coordinator.TrajectoryEnvelopeCoordinatorROS;
import se.oru.coordination.coordinator.util.IliadMissions;

public class NCFMDemoMS2MainNode extends AbstractNodeMain {

	private List<Integer> robotIDs = null;
	private HashMap<Integer,Boolean> activeRobots = new HashMap<Integer,Boolean>();
	private HashMap<Integer,Pose> initialLocations = new HashMap<Integer,Pose>();
	private ConnectedNode node = null;
	private HashMap<Integer,Integer> robotIDstoGoalIDs = new HashMap<Integer,Integer>();
	private TrajectoryEnvelopeCoordinatorROS tec = null;
	private Coordinate[] footprintCoords = null;
	private int CONTROL_PERIOD = 1000;
	private double TEMPORAL_RESOLUTION = 1000.0;
	private double MAX_ACCEL = 1.0;
	private double MAX_VEL = 4.0;
	private String missionsFile = null;
	private HashMap<Integer,Integer> robotID2MissionNumber = null;
	private HashMap<Integer,Boolean> isTaskComputing = null;
	
	private HashMap<Integer,Boolean> robotsAlive;
	private boolean loadedMissions = false;
	
	private boolean ignorePickItems = true;
	private boolean copyGoalOperationToStartoperation = false;
	
	private String reportTopic = "report";
	private String mapFrameID = "map";
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("coordinator");
	}

	private void setupActivateServices() {
		node.newServiceServer("coordinator/activate", orunav_msgs.Trigger._TYPE, new ServiceResponseBuilder<orunav_msgs.TriggerRequest, orunav_msgs.TriggerResponse>() {
			@Override
			public void build(orunav_msgs.TriggerRequest arg0, orunav_msgs.TriggerResponse arg1) throws ServiceException {
				System.out.println(">>>>>>>>>>>>>> ACTIVATING Robot" + arg0.getRobotID());
				activeRobots.put(arg0.getRobotID(),true);
			}
		});
		node.newServiceServer("coordinator/deactivate", orunav_msgs.Trigger._TYPE, new ServiceResponseBuilder<orunav_msgs.TriggerRequest, orunav_msgs.TriggerResponse>() {
			@Override
			public void build(orunav_msgs.TriggerRequest arg0, orunav_msgs.TriggerResponse arg1) throws ServiceException {
				System.out.println(">>>>>>>>>>>>>> DEACTIVATING Robot" + arg0.getRobotID());
				activeRobots.put(arg0.getRobotID(),false);			}
		});
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {

		this.node = connectedNode;
		
		while (true) {
			try {
				connectedNode.getCurrentTime();
				break;
			}
			catch(NullPointerException e) { }
		}

		//read parameters from launch file
		readParams();
			
		// This CancellableLoop will be canceled automatically when the node shuts down.
		node.executeCancellableLoop(new CancellableLoop() {

			@Override
			protected void setup() {
				
				long origin = TimeUnit.NANOSECONDS.toMillis(node.getCurrentTime().totalNsecs());
				//Instantiate a trajectory envelope coordinator (with ROS support)
				tec = new TrajectoryEnvelopeCoordinatorROS(CONTROL_PERIOD, TEMPORAL_RESOLUTION, node);
				tec.addComparator(new Comparator<RobotAtCriticalSection> () {
					@Override
					public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
						CriticalSection cs = o1.getCriticalSection();
						RobotReport robotReport1 = o1.getRobotReport();
						RobotReport robotReport2 = o2.getRobotReport();
						return ((cs.getTe1Start()-robotReport1.getPathIndex())-(cs.getTe2Start()-robotReport2.getPathIndex()));
					}
					});
					tec.addComparator(new Comparator<RobotAtCriticalSection> () {
						@Override
						public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
							return(o2.getRobotReport().getRobotID()-o1.getRobotReport().getRobotID());
						}
					});
				
				//Need to setup infrastructure that maintains the representation
				tec.setupSolver(origin, origin+100000000L);
				
				//Start the thread that revises precedences at every period
				tec.startInference();
				
				//Setup a simple GUI (null means empty map, otherwise provide yaml file)
				//final JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
				final RVizVisualization viz = new RVizVisualization(node,mapFrameID);
				tec.setVisualization(viz);
							
				setupActivateServices();
				
				for (final int robotID : robotIDs) {
					
					//Set the footprint of the robots
					tec.setFootprint(robotID, footprintCoords);
					
					//Set the forward dynamic model for the robot so the coordinator
					//can estimate whether the robot can stop
					tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(robotID)));
					
					//Get all initial locations of robots (this is done once)
					Subscriber<orunav_msgs.RobotReport> subscriberInit = node.newSubscriber("robot"+robotID+"/"+reportTopic, orunav_msgs.RobotReport._TYPE);
					subscriberInit.addMessageListener(new MessageListener<orunav_msgs.RobotReport>() {
						@Override
						public void onNewMessage(orunav_msgs.RobotReport message) {
							if (!message.getStamp().isZero() && !initialLocations.containsKey(robotID)) {
								Quaternion quat = new Quaternion(message.getState().getPose().getOrientation().getX(), message.getState().getPose().getOrientation().getY(), message.getState().getPose().getOrientation().getZ(), message.getState().getPose().getOrientation().getW());
								Pose pose = new Pose(message.getState().getPose().getPosition().getX(), message.getState().getPose().getPosition().getY(), quat.getTheta());
								initialLocations.put(robotID, pose);
								//Place all robots in current positions
								tec.placeRobot(robotID, pose, null, "r"+robotID+"p");
								System.out.println("PLACED ROBOT " + robotID + " in " + pose);
								robotsAlive.put(robotID,true);
							}
						}
					});
					
					
					Subscriber<geometry_msgs.PoseStamped> subscriberGoal = node.newSubscriber("robot"+robotID+"/goal", geometry_msgs.PoseStamped._TYPE);
					subscriberGoal.addMessageListener(new MessageListener<geometry_msgs.PoseStamped>() {
						@Override
						public void onNewMessage(geometry_msgs.PoseStamped message) {
							Quaternion quat = new Quaternion(message.getPose().getOrientation().getX(), message.getPose().getOrientation().getY(), message.getPose().getOrientation().getZ(), message.getPose().getOrientation().getW());
							Pose goalPose = new Pose(message.getPose().getPosition().getX(), message.getPose().getPosition().getY(),quat.getTheta());
							Pose startPose = tec.getRobotReport(robotID).getPose();
							IliadMission mission = new IliadMission(robotID, null, "A", "B", startPose, goalPose, OPERATION_TYPE.NO_OPERATION, OPERATION_TYPE.NO_OPERATION, false);
							System.out.println("POSTED MISSION:\n" + mission.toXML());
							String postedGoalLog = System.getProperty("user.home")+File.separator+"posted_goals.xml";
							PrintWriter writer;
							try {
								writer = new PrintWriter(new FileOutputStream(new File(postedGoalLog), true));
								writer.println(mission.toXML());
					            writer.close();
							}
							catch (FileNotFoundException e) { e.printStackTrace(); } 
							callComputeTaskService(mission);
						}
					});
					
				}
				
			}

			@Override
			protected void loop() throws InterruptedException {

				boolean allRobotsAlive = true;
				for (int robotID : robotIDs) if (!robotsAlive.get(robotID)) allRobotsAlive = false;
				
				if (allRobotsAlive) {
					
					//This is done once
					if (!loadedMissions) {
						if (missionsFile != null) {
							loadedMissions = true;
							IliadMissions.loadIliadMissions(missionsFile);
							robotID2MissionNumber = new HashMap<Integer,Integer>();
							isTaskComputing = new HashMap<Integer,Boolean>();
							for (int robotID : robotIDs) {
								robotID2MissionNumber.put(robotID, 0);
								isTaskComputing.put(robotID, false);
							}
							//This is to ensure that the motion primitives have been loaded by the motion planner
							Thread.sleep(10000);
						}
					}
				
					//This is done at every cycle
					for (int robotID : robotIDs) {
						if (tec.isFree(robotID)) {
							if (!isTaskComputing.get(robotID) && activeRobots.get(robotID)) {
								ArrayList<Mission> missions = IliadMissions.getMissions(robotID);
								if (missions != null) {
									//TODO: Should check if robot is close to intended start pose instead
									//of overwriting it with current pose from RobotReport...
									int missionNumber = robotID2MissionNumber.get(robotID);
									robotID2MissionNumber.put(robotID,(missionNumber+1)%IliadMissions.getMissions(robotID).size());
									IliadMission mission = (IliadMission)missions.get(missionNumber);
									Pose startPose = tec.getRobotReport(robotID).getPose();
									mission.setFromPose(startPose);
									//Compute the path and add it
									//(we know adding will work because we checked that the robot is free)
									System.out.println("CALLING COMPUTE TASK ON MISSION " + mission);
									callComputeTaskService(mission);
								}
							}
						}
					}
				}
								
				Thread.sleep(1000);
			}
		});
	}
	
	@SuppressWarnings("unchecked")
	private void readParams() {
		ParameterTree params = node.getParameterTree();
		footprintCoords = new Coordinate[4];
		try {
			footprintCoords[0] = new Coordinate(params.getDouble("/" + node.getName() + "/footprint_rear_left_x"),params.getDouble("/" + node.getName() + "/footprint_rear_left_y"));
			footprintCoords[1] = new Coordinate(params.getDouble("/" + node.getName() + "/footprint_rear_right_x"),params.getDouble("/" + node.getName() + "/footprint_rear_right_y"));
			footprintCoords[2] = new Coordinate(params.getDouble("/" + node.getName() + "/footprint_front_right_x"),params.getDouble("/" + node.getName() + "/footprint_front_right_y"));
			footprintCoords[3] = new Coordinate(params.getDouble("/" + node.getName() + "/footprint_front_left_x"),params.getDouble("/" + node.getName() + "/footprint_front_left_y"));
			robotIDs = (List<Integer>) params.getList("/" + node.getName() + "/robot_ids");
			for (Integer robotID : robotIDs) activeRobots.put(robotID, false);
			ArrayList<Integer> defaultList = new ArrayList<Integer>();
			defaultList.add(-1);
			List<Integer> activeIDs = (List<Integer>) params.getList("/" + node.getName() + "/active_robot_ids", defaultList);
			//If param was not specified, assume all robots are active
			if (activeIDs.contains(-1)) {
				activeIDs = new ArrayList<Integer>();
				for (int robotID : robotIDs) activeIDs.add(robotID);
			}
			for (Integer active : activeIDs) activeRobots.put(active, true);
			CONTROL_PERIOD = params.getInteger("/" + node.getName() + "/control_period");
			TEMPORAL_RESOLUTION = params.getDouble("/" + node.getName() + "/temporal_resolution");
			MAX_ACCEL = params.getDouble("/" + node.getName() + "/forward_model_max_accel");
			MAX_VEL = params.getDouble("/" + node.getName() + "/forward_model_max_vel");
			robotsAlive = new HashMap<Integer,Boolean>();
			ignorePickItems = params.getBoolean("/" + node.getName() + "/ignore_pick_items",true);
			copyGoalOperationToStartoperation = params.getBoolean("/" + node.getName() + "/copy_goal_operation_to_start_operation",false);
			for (int robotID : robotIDs) robotsAlive.put(robotID,false);
			if (params.has("/" + node.getName() + "/missions_file")) missionsFile = params.getString("/" + node.getName() + "/missions_file");
			this.reportTopic = params.getString("/" + node.getName() + "/report_topic", "report");
			this.mapFrameID = params.getString("/" + node.getName() + "/map_frame_id", "map");
		}
		catch (org.ros.exception.ParameterNotFoundException e) {
			System.out.println("== Parameter not found ==");
			e.printStackTrace();
		}
	}
	
	private void callComputeTaskService(final IliadMission iliadMission) {
		
		isTaskComputing.put(iliadMission.getRobotID(), true);
		ServiceClient<ComputeTaskRequest, ComputeTaskResponse> serviceClient;
		try { serviceClient = node.newServiceClient("/robot" + iliadMission.getRobotID() + "/compute_task", ComputeTask._TYPE); }
		catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
		
		if (!robotIDstoGoalIDs.keySet().contains(iliadMission.getRobotID())) robotIDstoGoalIDs.put(iliadMission.getRobotID(), 1);
		else robotIDstoGoalIDs.put(iliadMission.getRobotID(), robotIDstoGoalIDs.get(iliadMission.getRobotID())+1);
		final int goalID = robotIDstoGoalIDs.get(iliadMission.getRobotID());
		
		final ComputeTaskRequest request = serviceClient.newMessage();
		request.setStartFromCurrentState(true);
		RobotTarget rt = node.getTopicMessageFactory().newFromType(RobotTarget._TYPE);
		orunav_msgs.PoseSteering ps = node.getTopicMessageFactory().newFromType(orunav_msgs.PoseSteering._TYPE);
		geometry_msgs.Pose gpose = node.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
		geometry_msgs.Point point = node.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
		point.setX(iliadMission.getToPose().getX());
		point.setY(iliadMission.getToPose().getY());
		point.setZ(0.0);
		gpose.setPosition(point);
		Quaternion metaCSPQuat = new Quaternion(iliadMission.getToPose().getTheta());
		geometry_msgs.Quaternion quat = node.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);
		quat.setX(metaCSPQuat.getX());
		quat.setY(metaCSPQuat.getY());
		quat.setZ(metaCSPQuat.getZ());
		quat.setW(metaCSPQuat.getW());
		gpose.setOrientation(quat);
		ps.setPose(gpose);
		ps.setSteering(0.0);
		rt.setGoal(ps);
		rt.setRobotId(iliadMission.getRobotID());
		rt.setTaskId(goalID);
		rt.setGoalId(goalID);
		request.setTarget(rt);
		serviceClient.call(request, new ServiceResponseListener<ComputeTaskResponse>() {

			@Override
			public void onFailure(RemoteException arg0) {
				System.out.println("FAILED to call ComputeTask service for robot" + iliadMission.getRobotID() + " (goalID: " + goalID + ")");				
			}

			@Override
			public void onSuccess(ComputeTaskResponse arg0) {
				System.out.println("Successfully called ComputeTask service for robot" + iliadMission.getRobotID() + " (goalID: " + goalID + ")");
				
				ArrayList<PoseSteering> path = new ArrayList<PoseSteering>();
				for (int i = 0; i < arg0.getTask().getPath().getPath().size(); i++) {
					orunav_msgs.PoseSteering onePS = arg0.getTask().getPath().getPath().get(i);
					Quaternion quat = new Quaternion(onePS.getPose().getOrientation().getX(), onePS.getPose().getOrientation().getY(), onePS.getPose().getOrientation().getZ(), onePS.getPose().getOrientation().getW());
					PoseSteering ps = new PoseSteering(onePS.getPose().getPosition().getX(), onePS.getPose().getPosition().getY(), quat.getTheta(), onePS.getSteering());
					path.add(ps);
				}
				PoseSteering[] pathArray = path.toArray(new PoseSteering[path.size()]);
				iliadMission.setPath(pathArray);
				
				//Operations used by the current execution service
				Operation goalOp = node.getTopicMessageFactory().newFromType(Operation._TYPE);
				goalOp.setOperation(iliadMission.getGoalOperation().ordinal());
				//goalOp.setOperation(Operation.NO_OPERATION);
				if (iliadMission.getGoalOperation().equals(OPERATION_TYPE.PICK_ITEMS)) {
					if (ignorePickItems) {
						System.out.println("Ignoring PICK_ITEMS operation (see launch file)");
						goalOp.setOperation(Operation.NO_OPERATION);
					}
					orunav_msgs.IliadItemArray iliadItemArrayMsg = node.getTopicMessageFactory().newFromType(orunav_msgs.IliadItemArray._TYPE);
					ArrayList<orunav_msgs.IliadItem> itemList = new ArrayList<orunav_msgs.IliadItem>();
					for (IliadItem item : iliadMission.getItems()) {
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

				tec.setCurrentTask(arg0.getTask().getTarget().getRobotId(), arg0.getTask());
				tec.addMissions(iliadMission);
				//tec.computeCriticalSections();
				//tec.startTrackingAddedMissions();
				isTaskComputing.put(arg0.getTask().getTarget().getRobotId(), false);
			}
			
		});
	}

}
