package se.oru.coordination.coordinator.calibration;

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

import se.oru.coordination.coordination_oru.*;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.RVizVisualization;
import se.oru.coordination.coordinator.ros_coordinator.IliadMission;
import se.oru.coordination.coordinator.ros_coordinator.TrajectoryEnvelopeCoordinatorROS;
import se.oru.coordination.coordinator.ros_coordinator.orkla.ComputeIliadTaskServiceMotionPlanner;

import se.oru.coordination.coordinator.util.IliadMissions;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;
import java.util.concurrent.TimeUnit;


public class CalibrationMainNode extends AbstractNodeMain {
	// Coordination related variables
	private HashMap<Integer,Boolean> activeRobots = new HashMap<Integer,Boolean>();
	private HashMap<Integer,Pose> initialLocations = new HashMap<Integer,Pose>();
	private TrajectoryEnvelopeCoordinatorROS tec = null;
	private int CONTROL_PERIOD = 1000;
	private double TEMPORAL_RESOLUTION = 1000.0;
	
	// Mission related variables
	private String missionsFile = null;
	private HashMap<Integer,Boolean> robotsAlive;
	private ConcurrentMap<Integer,Boolean> repeatMission = new ConcurrentHashMap<Integer,Boolean>();
	private HashMap<Integer, Double> curveWidthInOdom = null;
	private HashMap<Integer, Double> curveHeightInOdom = null;
	
	// Robot related variables
	private List<Integer> robotIDs = null;
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
				activeRobots.put(arg0.getRobotID(), true);
			}
		});
		node.newServiceServer("coordinator/deactivate", orunav_msgs.Trigger._TYPE, new ServiceResponseBuilder<orunav_msgs.TriggerRequest, orunav_msgs.TriggerResponse>() {
			@Override
			public void build(orunav_msgs.TriggerRequest arg0, orunav_msgs.TriggerResponse arg1) throws ServiceException {
				System.out.print(ANSI_BLUE + ">>>>>>>>>>>>>> DEACTIVATING Robot" + arg0.getRobotID());
				System.out.println(ANSI_RESET);
				activeRobots.put(arg0.getRobotID(), false);
			}
		});
		node.newServiceServer("coordinator/enable_calibration", orunav_msgs.SetBool._TYPE, new ServiceResponseBuilder<orunav_msgs.SetBoolRequest, orunav_msgs.SetBoolResponse>() {
			@Override
			public void build(orunav_msgs.SetBoolRequest arg0, orunav_msgs.SetBoolResponse arg1) throws ServiceException {
				System.out.print(ANSI_BLUE + ">>>>>>>>>>>>>> ENABLE CALIBRATION for Robot" + arg0.getRobotID() + ", data: " + arg0.getData() + ".");
				System.out.println(ANSI_RESET);
				repeatMission.put(arg0.getRobotID(), arg0.getData());
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
				tec.setNetworkParameters(0.0, 1000, 0.0); //Set the upper bound of the transmission delay to 4000 ms (necessary in practice to break via abort service)
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
				
				//Path are planned in odom frame. No coordination is possible.
				tec.setFakeCoordination(true);
				
				//Setup a simple GUI (null means empty map, otherwise provide yaml file)
				//final JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
				final RVizVisualization viz = new RVizVisualization(node, mapFrameID);
				tec.setVisualization(viz);

				//Load or generate the path
				if (missionsFile != null) Missions.loadScenario(missionsFile);
				else generateCalibrationPaths();

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

					//Set the forward dynamic model for the robot so the coordinator
					//can estimate whether the robot can stop
					tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(max_accel.get(robotID), max_vel.get(robotID), tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(robotID)));

					//Get all initial locations of robots (this is done once)
					Subscriber<orunav_msgs.RobotReport> subscriberInit = node.newSubscriber("/robot"+robotID+"/"+reportTopic, orunav_msgs.RobotReport._TYPE);
					subscriberInit.addMessageListener(new MessageListener<orunav_msgs.RobotReport>() {
						@Override
						public void onNewMessage(orunav_msgs.RobotReport message) {
							if (!message.getStamp().isZero() && !initialLocations.containsKey(robotID) && activeRobots.get(robotID)) {
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
					repeatMission.put(robotID, false);
				}
			}

			@Override
			protected void loop() throws InterruptedException {
				boolean allRobotsAlive = true;
				for (int robotID : robotIDs) 
					if (!robotsAlive.get(robotID)) {
						System.out.print(ANSI_RED + "ROBOT " + robotID + " is not active.");
						System.out.println(ANSI_RESET);
						allRobotsAlive = false;
					}
				
				if (allRobotsAlive) {					
					//Start the thread that revises precedences at every period
					if (!tec.isStartedInference()) tec.startInference();
					for (final int robotID : robotIDs) {
						if (tec.isFree(robotID)) {		
							if (IliadMissions.hasMissions(robotID) && repeatMission.get(robotID) && activeRobots.get(robotID)) {
								final IliadMission m = (IliadMission)IliadMissions.peekMission(robotID);
								tec.addMissions(m);
							}
						}
					}
				}
				Thread.sleep(1000);
			}
		});
		System.out.print(ANSI_GREEN + "STARTED AUTOMATIC-CALIBRATION MISSION DISPATCHING!");
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
			System.out.print(ANSI_BLUE + "Got RobotIDs ... ");
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

			for (Integer robotID : robotIDs) {

				String footprintParamNames[] = {
						"/robot" + robotID + "/footprint/rear_left_x", "/robot" + robotID + "/footprint/rear_left_y",
						"/robot" + robotID + "/footprint/rear_right_x", "/robot" + robotID + "/footprint/rear_right_y",
						"/robot" + robotID + "/footprint/front_right_x", "/robot" + robotID + "/footprint/front_right_y",
						"/robot" + robotID + "/footprint/front_left_x", "/robot" + robotID + "/footprint/front_left_y"
				};
				String maxAccelParamName = "/robot" + robotID + "/execution/max_acc";
				String maxVelParamName = "/robot" + robotID + "/execution/max_vel";
				String curveWidth = "/robot" + robotID + "/calibration/curve_width_in_odom_frame";
				String curveHeight = "/robot" + robotID + "/calibration/curve_height_in_odom_frame";

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
						!params.has(maxAccelParamName)      || !params.has(maxVelParamName) ||
						!params.has(curveWidth)      || !params.has(curveHeight)) {
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
				curveWidthInOdom.put(robotID, params.getDouble(curveWidth));
				curveHeightInOdom.put(robotID, params.getDouble(curveHeight));

				System.out.print(ANSI_BLUE + "Got all robot-specific params ... ");
				System.out.println(ANSI_RESET);				
			}

			CONTROL_PERIOD = params.getInteger("/" + node.getName() + "/control_period");
			TEMPORAL_RESOLUTION = params.getDouble("/" + node.getName() + "/temporal_resolution");
			
			robotsAlive = new HashMap<Integer,Boolean>();
			for (int robotID : robotIDs) robotsAlive.put(robotID, false);
			if (params.has("/" + node.getName() + "/missions_file")) missionsFile = params.getString("/" + node.getName() + "/missions_file");
			this.reportTopic = params.getString("/" + node.getName() + "/report_topic", "report");
			this.mapFrameID = params.getString("/" + node.getName() + "/map_frame_id", "map");

		}
		catch (org.ros.exception.ParameterNotFoundException e) {
			System.out.print(ANSI_RED + "== Parameter not found ==");
			System.out.println(ANSI_RESET);
			e.printStackTrace();
		}
	}
	
	private void generateCalibrationPaths() {
		for (final int robotID : robotIDs) {
			tec.setFootprint(robotID, footprintCoords.get(robotID));
			ComputeIliadTaskServiceMotionPlanner mp = new ComputeIliadTaskServiceMotionPlanner(robotID, node, tec);
			mp.setFootprint(footprintCoords.get(robotID));
			mp.clearObstacles();

			//Generate the eight-shaped curve
			Pose[] locations = new Pose[7];
			locations[0] = new Pose(0, 0, 0);
			locations[1] = new Pose(0.5*curveHeightInOdom.get(robotID), -0.25*curveWidthInOdom.get(robotID), -Math.PI/2);					
			locations[2] = new Pose(-0.5*curveHeightInOdom.get(robotID), -1.5*curveWidthInOdom.get(robotID), -Math.PI/2);
			locations[3] = new Pose(0, -curveWidthInOdom.get(robotID), 0);
			locations[4] = new Pose(0.5*curveHeightInOdom.get(robotID), -0.75*curveWidthInOdom.get(robotID), Math.PI/2);
			locations[5] = new Pose(-0.5*curveHeightInOdom.get(robotID), -0.25*curveWidthInOdom.get(robotID), Math.PI/2);
			locations[6] = new Pose(0, 0, 0);
	
			//start motion planner (piece-by-piece)
			IliadMission[] m = new IliadMission[locations.length-1];
			for (int i = 0; i < locations.length-1; i++) {
				mp.setStart(locations[i]);
				mp.setGoals(locations[i+1]);
				if (!mp.plan()) throw new Error("No path found");
				m[i] = new IliadMission(robotID, mp.getPath(), locations[i].toString(), locations[i+1].toString(), locations[i], locations[i+1], false);
			}
			Missions.concatenateMissions(m);
		}
	}

}