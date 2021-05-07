package se.oru.coordination.coordinator.ros_coordinator.orkla;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Quaternion;
import org.ros.concurrent.CancellableLoop;
import org.ros.exception.ServiceException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.topic.Subscriber;
import orunav_msgs.Trigger;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.*;
import se.oru.coordination.coordination_oru.util.RVizVisualization;
import se.oru.coordination.coordinator.ros_coordinator.IliadItem;
import se.oru.coordination.coordinator.ros_coordinator.IliadItem.ROTATION_TYPE;
import se.oru.coordination.coordinator.ros_coordinator.IliadMission;
import se.oru.coordination.coordinator.ros_coordinator.TrajectoryEnvelopeCoordinatorROS;
import se.oru.coordination.coordinator.ros_coordinator.IliadMission.OPERATION_TYPE;
import se.oru.coordination.coordinator.util.IliadMissions;
import java.util.HashSet;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.TimeUnit;


public class OrklaDemoMS4MainNode extends AbstractNodeMain {
	// Coordination related variables
	private HashMap<Integer,Boolean> activeRobots = new HashMap<Integer,Boolean>();
	private HashMap<Integer,Pose> initialLocations = new HashMap<Integer,Pose>();
	private TrajectoryEnvelopeCoordinatorROS tec = null;
	private int CONTROL_PERIOD = 1000;
	private double TEMPORAL_RESOLUTION = 1000.0;
	
	// Mission related variables
	private boolean ignorePickItems = false;
	private HashMap<Integer,Boolean> robotsAlive;
	
	// Robot related variables
	private List<Integer> robotIDs = null;
	private HashMap<Integer,Boolean> isTaskComputing = new HashMap<Integer,Boolean>();
	private ConcurrentHashMap<Integer,Boolean> taskComputingSucceed = new ConcurrentHashMap<Integer,Boolean>();
	private ConcurrentHashMap<Integer,Boolean> canDispatchNewTask = new ConcurrentHashMap<Integer,Boolean>();
	private HashMap<Integer, Coordinate[]> footprintCoords = null;
	private HashMap<Integer, Double> max_accel = null;
	private HashMap<Integer, Double> max_vel = null;
	
	// ROS related
	private String reportTopic = "report";
	private String mapFrameID = "map_laser2d";
	private ConnectedNode node = null;
	
	public static final String ANSI_BG_WHITE = "\u001B[47m";
	// Nice visualization
	public static final String ANSI_BLUE = "\u001B[34m" + ANSI_BG_WHITE;
	public static final String ANSI_GREEN = "\u001B[32m" + ANSI_BG_WHITE;
	public static final String ANSI_RED = "\u001B[31m" + ANSI_BG_WHITE;
	public static final String ANSI_RESET = "\u001B[0m";

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("coordinator");
	}

	private void setupServices() {
		node.newServiceServer("coordinator/activate", orunav_msgs.Trigger._TYPE, new ServiceResponseBuilder<orunav_msgs.TriggerRequest, orunav_msgs.TriggerResponse>() {
			@Override
			public void build(orunav_msgs.TriggerRequest arg0, orunav_msgs.TriggerResponse arg1) throws ServiceException {
				System.out.print(ANSI_BLUE + ">>>>>>>>>>>>>> ACTIVATING Robot" + arg0.getRobotID());
				System.out.println(ANSI_RESET);
				activeRobots.put(arg0.getRobotID(),true);
			}
		});
		node.newServiceServer("coordinator/deactivate", orunav_msgs.Trigger._TYPE, new ServiceResponseBuilder<orunav_msgs.TriggerRequest, orunav_msgs.TriggerResponse>() {
			@Override
			public void build(orunav_msgs.TriggerRequest arg0, orunav_msgs.TriggerResponse arg1) throws ServiceException {
				System.out.print(ANSI_BLUE + ">>>>>>>>>>>>>> DEACTIVATING Robot" + arg0.getRobotID());
				System.out.println(ANSI_RESET);
				activeRobots.put(arg0.getRobotID(),false);
			}
		});
		node.newServiceServer("coordinator/update_task", orunav_msgs.UpdateTask._TYPE, new ServiceResponseBuilder<orunav_msgs.UpdateTaskRequest, orunav_msgs.UpdateTaskResponse>() {
			@Override
			public void build(orunav_msgs.UpdateTaskRequest arg0, orunav_msgs.UpdateTaskResponse arg1) throws ServiceException {
				
				int rid = arg0.getTask().getTarget().getRobotId();
				if (!robotIDs.contains(rid)) {
					System.out.print(ANSI_BLUE + ">>>>>>>>>>>>>> Ignoring task update request as there is no robot with ID = " + rid);
					System.out.println(ANSI_RESET);
					return;
				}
				
				System.out.print(ANSI_BLUE + ">>>>>>>>>>>>>> Updating task for Robot" + rid + ": new path piece size " + arg0.getTask().getPath().getPath().size());
				System.out.println(ANSI_RESET);
				
				synchronized(tec.getSolver()) {
					
					//Get old path
					PoseSteering[] oldP = tec.getCurrentTrajectoryEnvelope(rid).getTrajectory().getPoseSteering();
					//Get the new path from the message
					List<orunav_msgs.PoseSteering> newPath = arg0.getTask().getPath().getPath();
					PoseSteering[] newP = null;
					boolean concatenatePaths = true;
					int offset = 0;
					
					if (arg0.getTask().getUpdate()) {					
						newP = new PoseSteering[oldP.length + newPath.size()];
						
						//Concatenate the new path to the old path...
						for (int i = 0; i < oldP.length; i++) newP[i] = oldP[i];
						offset = oldP.length;
					}
					//FIXME concatenation at the end of the path
					else {
						RobotReport rep = tec.getRobotReport(rid);
						if (rep.getPathIndex() == oldP.length-1) {
							//The robot has traversed the overall old path. We may start a new path.
							//This allows e.g., to move from a LOAD_DETECT to a LOAD mission
							concatenatePaths = false;
						}
						newP =  new PoseSteering[newPath.size()];
					}
					
					for (int i = 0; i < newPath.size(); i++) {
						orunav_msgs.PoseSteering ps = newPath.get(i);
						Quaternion quatOrientation = new Quaternion(ps.getPose().getOrientation().getX(),ps.getPose().getOrientation().getY(),ps.getPose().getOrientation().getZ(),ps.getPose().getOrientation().getW());
						newP[i+offset] = new PoseSteering(ps.getPose().getPosition().getX(), ps.getPose().getPosition().getY(), quatOrientation.getTheta(), ps.getSteering());
					}
					
					//Update operations too...
					tec.getCurrentTracker(rid).setOperations(arg0.getTask().getTarget().getStartOp(), arg0.getTask().getTarget().getGoalOp());
					
					//... and tell the coordinator to update the current task (only information related to the current goal) ...
					orunav_msgs.Task currentTask = tec.getCurrentTask(rid);
					currentTask.getTarget().setGoal(arg0.getTask().getTarget().getGoal());
					currentTask.getTarget().setGoalId(arg0.getTask().getTarget().getGoalId());
					currentTask.getTarget().setGoalOp(arg0.getTask().getTarget().getGoalOp());
					tec.setCurrentTask(rid, currentTask);					
					
					//... and tell the coordinator to replace the path
					tec.replacePath(rid, newP, oldP.length-1, concatenatePaths, new HashSet<Integer>(rid));				

					System.out.println("Last communicated start pose: (x: " + tec.getCurrentTask(rid).getTarget().getStart().getPose().getPosition().getX() + ", y: "
							+ tec.getCurrentTask(rid).getTarget().getStart().getPose().getPosition().getY() + ").");
					
					System.out.print(ANSI_BLUE + ": new path size " + newP.length);
					System.out.println(ANSI_RESET);
				}
				
			}
		});
		node.newServiceServer("coordinator/abort", orunav_msgs.Trigger._TYPE, new ServiceResponseBuilder<orunav_msgs.TriggerRequest, orunav_msgs.TriggerResponse>() {
			@Override
			public void build(final orunav_msgs.TriggerRequest arg0, final orunav_msgs.TriggerResponse arg1) throws ServiceException {
				System.out.println(ANSI_RED + ">>>>>>>>>>>>>> ABORTING Robot" + arg0.getRobotID());
				System.out.println(ANSI_RESET);
				if (tec.isFree(arg0.getRobotID()) && !isTaskComputing.get(arg0.getRobotID())) {
					if (IliadMissions.hasMissions(arg0.getRobotID())) {
						IliadMissions.dequeueMission(arg0.getRobotID());
						System.out.println("Mission deleted before planning started.");
					}
					arg1.setSuccess(true);
					System.out.println("No missions to delete.");
					return;
				} 
				if (isTaskComputing.get(arg0.getRobotID())) {
					do { 
						canDispatchNewTask.put(arg0.getRobotID(), false);
						try { Thread.sleep(200); } 
						catch (InterruptedException e) {}
					}
					while (isTaskComputing.get(arg0.getRobotID()));
					if (!taskComputingSucceed.get(arg0.getRobotID())) {
						arg1.setSuccess(true);
						System.out.println("Mission not assigned");
						return;
					}
					do { 
						try { Thread.sleep(200); } 
						catch (InterruptedException e) {}
					}
					while (!tec.isDriving(arg0.getRobotID()));
				}
				if (tec.truncateEnvelope(arg0.getRobotID())) {
					//Update operations too ... (ATTENTION)
					tec.getCurrentTask(arg0.getRobotID()).getTarget().getGoalOp().setOperation(OPERATION_TYPE.NO_OPERATION.ordinal());
					tec.getCurrentTracker(arg0.getRobotID()).setOperations(tec.getCurrentTask(arg0.getRobotID()).getTarget().getStartOp(), tec.getCurrentTask(arg0.getRobotID()).getTarget().getGoalOp());
					arg1.setSuccess(true);
					System.out.println("Mission aborted after assignment.");
					return;
				}
				arg1.setSuccess(false);
				System.out.println("Mission cannot be aborted.");
			}
		});
		node.newServiceServer("coordinator/replan", orunav_msgs.Trigger._TYPE, new ServiceResponseBuilder<orunav_msgs.TriggerRequest, orunav_msgs.TriggerResponse>() {
			@Override
			public void build(orunav_msgs.TriggerRequest arg0, orunav_msgs.TriggerResponse arg1) throws ServiceException {
				System.out.println(ANSI_BLUE + ">>>>>>>>>>>>>> REPLANNING Robot" + arg0.getRobotID());
				System.out.println(ANSI_RESET);
				arg1.setSuccess(tec.replanEnvelope(arg0.getRobotID()));
			};
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

		System.out.print(ANSI_BLUE + "ALL PARAMETERS REQUIRED FOR THE COORDINATOR WERE READ SUCCESSFULLY!");
		System.out.println(ANSI_RESET);

		// This CancellableLoop will be canceled automatically when the node shuts down.
		node.executeCancellableLoop(new CancellableLoop() {

			@Override
			protected void setup() {

				long origin = TimeUnit.NANOSECONDS.toMillis(node.getCurrentTime().totalNsecs());
				//Instantiate a trajectory envelope coordinator (with ROS support)
				tec = new TrajectoryEnvelopeCoordinatorROS(CONTROL_PERIOD, TEMPORAL_RESOLUTION, node);
				tec.setNetworkParameters(0.0, 4000, 0.0); //Set the upper bound of the transmission delay to 4000 ms (necessary in practice to break via abort service)
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
				tec.setYieldIfParking(true);
				
				//Setup a simple GUI (null means empty map, otherwise provide yaml file)
				//final JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
				final RVizVisualization viz = new RVizVisualization(node, mapFrameID);
				tec.setVisualization(viz);

				//Sleep to allow loading of motion prims
				try {
					Thread.sleep(10000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}

				setupServices();

				for (final int robotID : robotIDs) {
					tec.setFootprint(robotID, footprintCoords.get(robotID));
					ComputeIliadTaskServiceMotionPlanner mp = new ComputeIliadTaskServiceMotionPlanner(robotID, node, tec);
					mp.setFootprint(footprintCoords.get(robotID));
					mp.setIgnorePickItems(ignorePickItems);
					tec.setMotionPlanner(robotID, mp);
					isTaskComputing.put(robotID, false);


					//Set the forward dynamic model for the robot so the coordinator
					//can estimate whether the robot can stop
					tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(max_accel.get(robotID), max_vel.get(robotID), tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(robotID)));

					//Get all initial locations of robots (this is done once)
					Subscriber<orunav_msgs.RobotReport> subscriberInit = node.newSubscriber("/robot"+robotID+"/"+reportTopic, orunav_msgs.RobotReport._TYPE);
					subscriberInit.addMessageListener(new MessageListener<orunav_msgs.RobotReport>() {
						@Override
						public void onNewMessage(orunav_msgs.RobotReport message) {
							if (!message.getStamp().isZero() && !initialLocations.containsKey(robotID)) {
								Quaternion quat = new Quaternion(message.getState().getPose().getOrientation().getX(), message.getState().getPose().getOrientation().getY(), message.getState().getPose().getOrientation().getZ(), message.getState().getPose().getOrientation().getW());
								Pose pose = new Pose(message.getState().getPose().getPosition().getX(), message.getState().getPose().getPosition().getY(), quat.getTheta());
								try {
									Thread.sleep(4000);
								} catch (InterruptedException e) {
									// TODO Auto-generated catch block
									e.printStackTrace();
								}
								initialLocations.put(robotID, pose);
								//Place all robots in current positions
								tec.placeRobot(robotID, pose, null, "r"+robotID+"p");
								System.out.print(ANSI_BLUE + "PLACED ROBOT " + robotID + " in " + pose);
								System.out.println(ANSI_RESET);
								robotsAlive.put(robotID,true);
							}
						}
					});

					//FIXME Name of the locations ... etc.
					Subscriber<orunav_msgs.RobotTarget> subscriberGoal = node.newSubscriber("robot"+robotID+"/robot_target", orunav_msgs.RobotTarget._TYPE);
					subscriberGoal.addMessageListener(new MessageListener<orunav_msgs.RobotTarget>() {
						@Override
						public void onNewMessage(orunav_msgs.RobotTarget message) {
							Quaternion quat = new Quaternion(message.getGoal().getPose().getOrientation().getX(), message.getGoal().getPose().getOrientation().getY(), message.getGoal().getPose().getOrientation().getZ(), message.getGoal().getPose().getOrientation().getW());
							Pose goalPose = new Pose(message.getGoal().getPose().getPosition().getX(), message.getGoal().getPose().getPosition().getY(),quat.getTheta());
							Pose startPose = tec.getRobotReport(robotID).getPose();
							IliadMission mission = null;
							if (message.getGoalOp().getOperation() == OPERATION_TYPE.PICK_ITEMS.ordinal()) {
								ArrayList<IliadItem> items = new ArrayList<IliadItem>();
								for (orunav_msgs.IliadItem item : message.getGoalOp().getItemlist().getItems()) items.add(new IliadItem(item.getName(), item.getPosition().getX(), item.getPosition().getY(), item.getPosition().getZ(), ROTATION_TYPE.values()[item.getRotationType()]));
								IliadItem[] itemsArray = items.toArray(new IliadItem[items.size()]);
								mission = new IliadMission(robotID, null, "A", "B", startPose, goalPose, OPERATION_TYPE.values()[message.getStartOp().getOperation()], false, itemsArray);
							}
							else mission = new IliadMission(robotID, null, "A", "B", startPose, goalPose, OPERATION_TYPE.values()[message.getStartOp().getOperation()], OPERATION_TYPE.values()[message.getGoalOp().getOperation()],false);
							IliadMissions.enqueueMission(mission);
							System.out.println("POSTED MISSION:\n" + mission.toXML());
							String postedGoalLog = System.getProperty("user.home")+File.separator+"posted_goals.xml";
							PrintWriter writer;
							try {
								writer = new PrintWriter(new FileOutputStream(new File(postedGoalLog), true));
								writer.println(mission.toXML());
					            writer.close();
							}
							catch (FileNotFoundException e) { e.printStackTrace(); } 
						}
					});
					isTaskComputing.put(robotID, false);
				}
			}

			@Override
			protected void loop() throws InterruptedException {
				boolean allRobotsAlive = true;
				for (int robotID : robotIDs) if (!robotsAlive.get(robotID)) allRobotsAlive = false;
				
				if (allRobotsAlive) {
					
					//Start the thread that revises precedences at every period
					if (!tec.isStartedInference()) tec.startInference();
					
					// Every cycle
					for (final int robotID : robotIDs) {
						if (tec.isFree(robotID)) {		
							if (IliadMissions.hasMissions(robotID) && !isTaskComputing.get(robotID) && activeRobots.get(robotID)) {
								final IliadMission m = (IliadMission)IliadMissions.dequeueMission(robotID);
								final ComputeIliadTaskServiceMotionPlanner mp = (ComputeIliadTaskServiceMotionPlanner)tec.getMotionPlanner(robotID);
								mp.clearObstacles();
								mp.setGoals(m.getToPose());
								mp.setStartOperation(m.getStartOperation());
								mp.setGoalOperation(m.getGoalOperation());
								mp.setPickItems(m.getItems());
								canDispatchNewTask.put(robotID, true);
								isTaskComputing.put(robotID, true);
								Thread planningThread = new Thread("Planning for robot " + robotID) {
									public void run() {
										System.out.print(ANSI_BLUE + ">>>>>>>>>>>>>>>> STARTED MOTION PLANNING for robot " + robotID);
										System.out.println(ANSI_RESET);	
										boolean succeed = mp.plan();
										if (succeed && canDispatchNewTask.get(robotID)) {
											m.setPath(mp.getPath());
											tec.addMissions(m);
											System.out.println("DISPATCHING MISSION:\n" + m.toXML());
										}
										System.out.print(ANSI_GREEN + "<<<<<<<<<<<<<<<< FINISHED MOTION PLANNING for robot " + robotID);
										System.out.println(ANSI_RESET);
										taskComputingSucceed.put(robotID, succeed && canDispatchNewTask.get(robotID));
										isTaskComputing.put(robotID, false);
									}
								};
								planningThread.start();
							}
						}
					}
				}

				Thread.sleep(1000);
			}
		});
		System.out.print(ANSI_GREEN + "COORDINATION INITIALIZED!");
		System.out.println(ANSI_RESET);
	}

	@SuppressWarnings("unchecked")
	private void readParams() {
		ParameterTree params = node.getParameterTree();
		footprintCoords = new HashMap<Integer, Coordinate[]>();
		max_vel = new HashMap<Integer, Double>();
		max_accel = new HashMap<Integer, Double>();

		// First we get the robot_id
		String robotIDsParamName = "/" + node.getName() + "/robot_ids";
		System.out.print(ANSI_BLUE + "Checking for robot_ids parameter.");
		System.out.println(ANSI_RESET);
		
		while(!params.has(robotIDsParamName)) {
			try {
				Thread.sleep(200);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}			
		}
		
		try {	
			robotIDs = (List<Integer>) params.getList(robotIDsParamName);
			System.out.print(ANSI_BLUE + "Got RobotIDs: " + robotIDs);
			System.out.println(ANSI_RESET);
			
			System.out.print(ANSI_BLUE + "Checking for active_robot_ids parameter.");
			System.out.println(ANSI_RESET);
			ArrayList<Integer> defaultList = new ArrayList<Integer>();
			defaultList.add(-1);
			List<Integer> activeIDs = (List<Integer>) params.getList("/" + node.getName() + "/active_robot_ids", defaultList);
			//If param was not specified, assume all robots are active
			if (activeIDs.contains(-1)) {
				System.out.print(ANSI_BLUE + "Assuming all robots are active since active_robot_ids parameter was not specified.");
				System.out.println(ANSI_RESET);
				for (int robotID : robotIDs) activeRobots.put(robotID, true);
			}
			else for (int robotID : activeIDs) if (robotIDs.contains(robotID)) activeRobots.put(robotID, true);
			
			System.out.print(ANSI_BLUE + "Got activeRobotIDs: " + activeRobots);
			System.out.println(ANSI_RESET);

			for (Integer robotID : robotIDs) {

				String footprintParamNames[] = {
						"/robot" + robotID + "/footprint/rear_left_x", "/robot" + robotID + "/footprint/rear_left_y",
						"/robot" + robotID + "/footprint/rear_right_x", "/robot" + robotID + "/footprint/rear_right_y",
						"/robot" + robotID + "/footprint/front_right_x", "/robot" + robotID + "/footprint/front_right_y",
						"/robot" + robotID + "/footprint/front_left_x", "/robot" + robotID + "/footprint/front_left_y"
				};
				String maxAccelParamName = "/robot" + robotID + "/execution/max_acc";
				String maxVelParamName = "/robot" + robotID + "/execution/max_vel";

				System.out.print(ANSI_BLUE + "Checking for these robot-specific parameters:\n" + 
						footprintParamNames[0] + "\n" + footprintParamNames[1] + "\n" +
						footprintParamNames[2] + "\n" + footprintParamNames[3] + "\n" +
						footprintParamNames[4] + "\n" +	footprintParamNames[5] + "\n" +
						footprintParamNames[6] + "\n" + footprintParamNames[7] + "\n" + 
						maxAccelParamName + "\n" + maxVelParamName);
				System.out.println(ANSI_RESET);

				while(!params.has(footprintParamNames[0]) || !params.has(footprintParamNames[1]) || 
						!params.has(footprintParamNames[2]) || !params.has(footprintParamNames[3]) || 
						!params.has(footprintParamNames[4]) || !params.has(footprintParamNames[5]) || 
						!params.has(footprintParamNames[6]) || !params.has(footprintParamNames[7]) ||
						!params.has(maxAccelParamName)      || !params.has(maxVelParamName)) {
					try {
						Thread.sleep(200);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}

				Coordinate[] thisFootprintCoords = new Coordinate[4];
				thisFootprintCoords[0] = new Coordinate(params.getDouble(footprintParamNames[0]),params.getDouble(footprintParamNames[1]));
				thisFootprintCoords[1] = new Coordinate(params.getDouble(footprintParamNames[2]),params.getDouble(footprintParamNames[3]));
				thisFootprintCoords[2] = new Coordinate(params.getDouble(footprintParamNames[4]),params.getDouble(footprintParamNames[5]));
				thisFootprintCoords[3] = new Coordinate(params.getDouble(footprintParamNames[6]),params.getDouble(footprintParamNames[7]));
				footprintCoords.put(robotID, thisFootprintCoords);		

				max_accel.put(robotID, params.getDouble(maxAccelParamName));
				max_vel.put(robotID, params.getDouble(maxVelParamName));

				System.out.print(ANSI_BLUE + "Got all robot-specific params ... ");
				System.out.println(ANSI_RESET);				
			}

			CONTROL_PERIOD = params.getInteger("/" + node.getName() + "/control_period");
			TEMPORAL_RESOLUTION = params.getDouble("/" + node.getName() + "/temporal_resolution");
			
			ignorePickItems = params.getBoolean("/" + node.getName() + "/ignore_pick_items", true);
			
			robotsAlive = new HashMap<Integer,Boolean>();
			for (int robotID : robotIDs) robotsAlive.put(robotID, false);

			this.reportTopic = params.getString("/" + node.getName() + "/report_topic", "report");
			this.mapFrameID = params.getString("/" + node.getName() + "/map_frame_id", "map");

		}
		catch (org.ros.exception.ParameterNotFoundException e) {
			System.out.print(ANSI_RED + "== Parameter not found ==");
			System.out.println(ANSI_RESET);
			e.printStackTrace();
		}
	}

}
