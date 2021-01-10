package se.oru.coordination.coordinator.ros_coordinator.generic;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.TimeUnit;

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

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.RVizVisualization;
import se.oru.coordination.coordinator.ros_coordinator.ComputeTaskServiceMotionPlanner;
import se.oru.coordination.coordinator.ros_coordinator.TrajectoryEnvelopeCoordinatorROS;

public class MainNode extends AbstractNodeMain {

	private List<Integer> robotIDs = null;
	private HashMap<Integer,Boolean> activeRobots = new HashMap<Integer,Boolean>();
	private HashMap<Integer,Pose> initialLocations = new HashMap<Integer,Pose>();
	private ConnectedNode node = null;
	private TrajectoryEnvelopeCoordinatorROS tec = null;
	private HashMap<Integer, Coordinate[]> footprintCoords = null;
	private HashMap<Integer, Double> max_accel = null;
	private HashMap<Integer, Double> max_vel = null;
	private int CONTROL_PERIOD = 1000;
	private double TEMPORAL_RESOLUTION = 1000.0;
	private String locationsFile = null;
	private String goalSequenceFile = null;
	private boolean repeatMissions = false;
	private String reportTopic = "report";
	private String mapFrameID = "map";
	private HashMap<Integer,Boolean> isPlanning = new HashMap<Integer,Boolean>();

	public static final String ANSI_BLUE = "\u001B[34m" + "\u001B[107m";
	public static final String ANSI_GREEN = "\u001B[32m" + "\u001B[107m";
	public static final String ANSI_RED = "\u001B[31m" + "\u001B[107m";
	public static final String ANSI_RESET = "\u001B[0m";

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("coordinator");
	}

	private void setupActivateServices() {
		node.newServiceServer("coordinator/activate", orunav_msgs.Abort._TYPE, new ServiceResponseBuilder<orunav_msgs.AbortRequest, orunav_msgs.AbortResponse>() {
			@Override
			public void build(orunav_msgs.AbortRequest arg0, orunav_msgs.AbortResponse arg1) throws ServiceException {
				System.out.print(ANSI_BLUE + ">>>>>>>>>>>>>> ACTIVATING Robot" + arg0.getRobotID());
				System.out.println(ANSI_RESET);
				activeRobots.put(arg0.getRobotID(),true);
			}
		});
		node.newServiceServer("coordinator/deactivate", orunav_msgs.Abort._TYPE, new ServiceResponseBuilder<orunav_msgs.AbortRequest, orunav_msgs.AbortResponse>() {
			@Override
			public void build(orunav_msgs.AbortRequest arg0, orunav_msgs.AbortResponse arg1) throws ServiceException {
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
				
				System.out.print(ANSI_BLUE + ">>>>>>>>>>>>>> Updating task for Robot" + rid);
				System.out.println(ANSI_RESET);
				
				//Get old path
				PoseSteering[] oldP = tec.getCurrentTrajectoryEnvelope(rid).getTrajectory().getPoseSteering();
				
				//Get the new path from the message
				List<orunav_msgs.PoseSteering> newPath = arg0.getTask().getPath().getPath();
				PoseSteering[] newP = new PoseSteering[oldP.length + newPath.size()];
				
				//Concatenate the new path to the old path...
				for (int i = 0; i < oldP.length; i++) {
					newP[i] = oldP[i];
				}
				for (int i = 0; i < newPath.size(); i++) {
					orunav_msgs.PoseSteering ps = newPath.get(i);
					Quaternion quatOrientation = new Quaternion(ps.getPose().getOrientation().getX(),ps.getPose().getOrientation().getY(),ps.getPose().getOrientation().getZ(),ps.getPose().getOrientation().getW());
					newP[i+oldP.length] = new PoseSteering(ps.getPose().getPosition().getX(), ps.getPose().getPosition().getY(), quatOrientation.getTheta(), ps.getSteering());
				}
				
				//Update operations too...
				tec.getCurrentTracker(rid).setOperations(arg0.getTask().getTarget().getStartOp(), arg0.getTask().getTarget().getGoalOp());
				
				//... and tell the coordinator to replace the path
				tec.replacePath(rid, newP);
				
			}
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
				
				//Start the thread that revises precedences at every period
				tec.startInference();

				//Setup a simple GUI (null means empty map, otherwise provide yaml file)
				//final JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
				final RVizVisualization viz = new RVizVisualization(node,mapFrameID);
				tec.setVisualization(viz);

				// Set the footprint of the robots
				// tec.setDefaultFootprint(footprintCoords);
				// FOOTPRINT IS SET IN LOOP BELOW!

				if (locationsFile != null) Missions.loadLocationAndPathData(locationsFile);
				//if (goalSequenceFile != null) readGoalSequenceFile();

				//Sleep to allow loading of motion prims
				try {
					Thread.sleep(10000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}

				setupActivateServices();


				for (final int robotID : robotIDs) {
					tec.setFootprint(robotID, footprintCoords.get(robotID));
					ComputeTaskServiceMotionPlanner mp = new ComputeTaskServiceMotionPlanner(robotID, node, tec);
					mp.setFootprint(footprintCoords.get(robotID));
					tec.setMotionPlanner(robotID, mp);
					isPlanning.put(robotID, false);


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
							}
						}
					});


					Subscriber<geometry_msgs.PoseStamped> subscriberGoal = node.newSubscriber("robot"+robotID+"/goal", geometry_msgs.PoseStamped._TYPE);
					subscriberGoal.addMessageListener(new MessageListener<geometry_msgs.PoseStamped>() {
						@Override
						public void onNewMessage(geometry_msgs.PoseStamped message) {
							Quaternion quat = new Quaternion(message.getPose().getOrientation().getX(), message.getPose().getOrientation().getY(), message.getPose().getOrientation().getZ(), message.getPose().getOrientation().getW());
							Pose pose = new Pose(message.getPose().getPosition().getX(), message.getPose().getPosition().getY(), quat.getTheta());
							Mission m = new Mission(robotID,"currentPose", pose.toString(), null, pose);
							Missions.enqueueMission(m);
						}
					});

				}
			}

			@Override
			protected void loop() throws InterruptedException {
				for (final int robotID : robotIDs) {
					if (tec.isFree(robotID)) {
						if (Missions.hasMissions(robotID) && !isPlanning.get(robotID)) {
							final Mission m = Missions.dequeueMission(robotID);
							final AbstractMotionPlanner mp = tec.getMotionPlanner(robotID);
							mp.clearObstacles();
							mp.setGoals(m.getToPose());
							isPlanning.put(robotID, true);
							Thread planningThread = new Thread("Planning for robot " + robotID) {
								public void run() {
									System.out.print(ANSI_BLUE + ">>>>>>>>>>>>>>>> STARTED MOTION PLANNING for robot " + robotID);
									System.out.println(ANSI_RESET);
									if (mp.plan()) {
										m.setPath(mp.getPath());
										tec.addMissions(m);
										//tec.computeCriticalSectionsAndStartTrackingAddedMission();
									}
									System.out.print(ANSI_GREEN + "<<<<<<<<<<<<<<<< FINISHED MOTION PLANNING for robot " + robotID);
									System.out.println(ANSI_RESET);
									isPlanning.put(robotID, false);
								}
							};
							planningThread.start();
						}
					}
				}

				// TODO: Fix reading from locations file/goal sequence file

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

			System.out.print(ANSI_BLUE + "Got RobotIDs ... ");
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

				activeRobots.put(robotID, false);
			}

			ArrayList<Integer> defaultList = new ArrayList<Integer>();
			defaultList.add(-1);
			List<Integer> activeIDs = (List<Integer>) params.getList("/" + node.getName() + "/active_robot_ids", defaultList);
			if (activeIDs.contains(-1)) {
				activeIDs = new ArrayList<Integer>();
				for (int robotID : robotIDs) activeIDs.add(robotID);
				for (Integer active : activeIDs) activeRobots.put(active, true);
			}
			CONTROL_PERIOD = params.getInteger("/" + node.getName() + "/control_period");
			TEMPORAL_RESOLUTION = params.getDouble("/" + node.getName() + "/temporal_resolution");

			locationsFile = params.getString("/" + node.getName() + "/locations_file", "NULL");
			goalSequenceFile = params.getString("/" + node.getName() + "/goal_sequence_file", "NULL");
			if (locationsFile.equals("NULL")) locationsFile = null;
			if (goalSequenceFile.equals("NULL")) goalSequenceFile = null;
			repeatMissions = params.getBoolean("/" + node.getName() + "/repeat_missions", false);
			this.reportTopic = params.getString("/" + node.getName() + "/report_topic", "report");
			this.mapFrameID = params.getString("/" + node.getName() + "/map_frame_id", "map");

		}
		catch (org.ros.exception.ParameterNotFoundException e) {
			System.out.print(ANSI_RED + "== Parameter not found ==");
			System.out.println(ANSI_RESET);
			e.printStackTrace();
		}
	}

//	private void readGoalSequenceFile() {
//		try {
//			Scanner in = new Scanner(new FileReader(goalSequenceFile));
//			while (in.hasNextLine()) {
//				String line = in.nextLine().trim();
//				if (line.length() != 0 && !line.startsWith("#")) {
//					String[] oneline = line.split(" |\t");
//					int robotID = Integer.parseInt(oneline[0]);
//					String goalLocation = oneline[1];
//					Mission m = new Mission(robotID, null, goalLocation, null, Missions.getLocation(goalLocation));
//					Missions.enqueueMission(m);
//					System.out.println(">>>>>>>>>>>>>>>>>>>>>>>>> ADDED MISSION " + m);
//				}
//			}
//			in.close();
//		}
//		catch (FileNotFoundException e) { e.printStackTrace(); }
//	}


}
