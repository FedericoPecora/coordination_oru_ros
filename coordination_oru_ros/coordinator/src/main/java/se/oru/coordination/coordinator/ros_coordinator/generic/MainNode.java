package se.oru.coordination.coordinator.ros_coordinator.generic;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.TimeUnit;

import org.metacsp.multi.spatioTemporal.paths.Pose;
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
	private Coordinate[] footprintCoords = null;
	private int CONTROL_PERIOD = 1000;
	private double TEMPORAL_RESOLUTION = 1000.0;
	private double MAX_ACCEL = 1.0;
	private double MAX_VEL = 4.0;
	private int ROBOT_REPORT_PERIOD = 1000;
	private int MAX_TX_DELAY = 0;
	private double MAX_PACKET_LOSS = 0;
	private double MAX_UNSAFETY_PROB = 1e-2;
	private String locationsFile = null;
	private String goalSequenceFile = null;
	private boolean repeatMissions = false;
	private String reportTopic = "report";
	private String mapFrameID = "map";
	private HashMap<Integer,Boolean> isPlanning = new HashMap<Integer,Boolean>();
    
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("coordinator");
	}
	
	private void setupActivateServices() {
		node.newServiceServer("coordinator/activate", orunav_msgs.Abort._TYPE, new ServiceResponseBuilder<orunav_msgs.AbortRequest, orunav_msgs.AbortResponse>() {
			@Override
			public void build(orunav_msgs.AbortRequest arg0, orunav_msgs.AbortResponse arg1) throws ServiceException {
				System.out.println(">>>>>>>>>>>>>> ACTIVATING Robot" + arg0.getRobotID());
				activeRobots.put(arg0.getRobotID(),true);
			}
		});
		node.newServiceServer("coordinator/deactivate", orunav_msgs.Abort._TYPE, new ServiceResponseBuilder<orunav_msgs.AbortRequest, orunav_msgs.AbortResponse>() {
			@Override
			public void build(orunav_msgs.AbortRequest arg0, orunav_msgs.AbortResponse arg1) throws ServiceException {
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
				
				//Set network configuration parameters
				tec.setNetworkParameters(MAX_PACKET_LOSS, MAX_TX_DELAY, MAX_UNSAFETY_PROB);
				
				//Need to setup infrastructure that maintains the representation
				tec.setupSolver(origin, origin+100000000L);
				tec.setYieldIfParking(true);
				
				//Setup a simple GUI (null means empty map, otherwise provide yaml file)
				//final JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
				final RVizVisualization viz = new RVizVisualization(node,mapFrameID);
				tec.setVisualization(viz);
				
				//Set the footprint of the robots
				tec.setDefaultFootprint(footprintCoords);
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
					
					ComputeTaskServiceMotionPlanner mp = new ComputeTaskServiceMotionPlanner(robotID, node, tec);
					mp.setFootprint(footprintCoords);
					tec.setMotionPlanner(robotID, mp);
					isPlanning.put(robotID, false);
					
					//Set the forward dynamic model for the robot so the coordinator
					//can estimate whether the robot can stop
					tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, TEMPORAL_RESOLUTION, ROBOT_REPORT_PERIOD));
					
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
								System.out.println("PLACED ROBOT " + robotID + " in " + pose);
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
					synchronized(tec.getSolver()) {
						if (tec.isFree(robotID)) {
							if (Missions.hasMissions(robotID) && !isPlanning.get(robotID)) {
								final Mission m = Missions.dequeueMission(robotID);
								final AbstractMotionPlanner mp = tec.getMotionPlanner(robotID);
								mp.clearObstacles();
								mp.setGoals(m.getToPose());
								isPlanning.put(robotID, true);
								Thread planningThread = new Thread("Planning for robot " + robotID) {
									public void run() {
										System.out.println(">>>>>>>>>>>>>>>> STARTED MOTION PLANNING for robot " + robotID);
										if (mp.plan()) {
											m.setPath(mp.getPath());
											tec.addMissions(m);
											tec.computeCriticalSectionsAndStartTrackingAddedMission();
										}
										System.out.println("<<<<<<<<<<<<<<<< FINISHED MOTION PLANNING for robot " + robotID);
										isPlanning.put(robotID, false);
									}
								};
								planningThread.start();
							}
						}
					}
				}
				
				// TODO: Fix reading from locations file/goal sequence file
				
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
			if (activeIDs.contains(-1)) {
				activeIDs = new ArrayList<Integer>();
				for (int robotID : robotIDs) activeIDs.add(robotID);
				for (Integer active : activeIDs) activeRobots.put(active, true);
			}
			CONTROL_PERIOD = params.getInteger("/" + node.getName() + "/control_period");
			TEMPORAL_RESOLUTION = params.getDouble("/" + node.getName() + "/temporal_resolution");
			MAX_ACCEL = params.getDouble("/" + node.getName() + "/forward_model_max_accel");
			MAX_VEL = params.getDouble("/" + node.getName() + "/forward_model_max_vel");
			ROBOT_REPORT_PERIOD = params.getInteger("/" + node.getName() + "/report_period");
			MAX_PACKET_LOSS = params.getDouble("/" + node.getName() + "/max_packet_loss_probability");
			MAX_TX_DELAY = params.getInteger("/" + node.getName() + "/max_transmission_delay");
			MAX_UNSAFETY_PROB = params.getDouble("/" + node.getName() + "/max_unsafety_probability");
			locationsFile = params.getString("/" + node.getName() + "/locations_file", "NULL");
			goalSequenceFile = params.getString("/" + node.getName() + "/goal_sequence_file", "NULL");
			if (locationsFile.equals("NULL")) locationsFile = null;
			if (goalSequenceFile.equals("NULL")) goalSequenceFile = null;
			repeatMissions = params.getBoolean("/" + node.getName() + "/repeat_missions", false);
			this.reportTopic = params.getString("/" + node.getName() + "/report_topic", "report");
			this.mapFrameID = params.getString("/" + node.getName() + "/map_frame_id", "map");

		}
		catch (org.ros.exception.ParameterNotFoundException e) {
			System.out.println("== Parameter not found ==");
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
