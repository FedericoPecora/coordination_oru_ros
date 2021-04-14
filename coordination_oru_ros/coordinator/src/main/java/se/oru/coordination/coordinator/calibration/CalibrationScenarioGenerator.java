package se.oru.coordination.coordinator.calibration;

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

import se.oru.coordination.coordinator.ros_coordinator.ComputeTaskServiceMotionPlanner;
import se.oru.coordination.coordinator.ros_coordinator.TrajectoryEnvelopeCoordinatorROS;
import se.oru.coordination.coordinator.util.IliadMissions;

public class CalibrationScenarioGenerator extends AbstractNodeMain {

	// Coordination related variables
	private HashMap<Integer,Boolean> activeRobots = new HashMap<Integer,Boolean>();
	
	// Mission related variables
	private String missionsFile = null;
	
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

				for (final int robotID : robotIDs) {
					//Instantiate a motion planner for this robot
					/*tec.setFootprint(robotID, footprintCoords.get(robotID));
					ComputeTaskServiceMotionPlanner mp = new ComputeTaskServiceMotionPlanner(robotID, node, tec);
					mp.setFootprint(footprintCoords.get(robotID));
					mp.clearObstacles();*/
					
					//Generate the waypoints for the Lemniscate curve
					//double xmax = 2*footprintCoords.get(robotID); [check here how to get the footprint coords]
					//ymax = a/2sqrt(2) -> just to know
					//t in (-pi/4; pi/4) U (3pi/4; 5pi/4)
					//x = xmax cost/(1+sin^2t)
					//y = x sint;
					//dy = x(1-2x^2-2y^2)/y(1+2x^2+2y^2);
					//theta = atan(dy);
					
					//mp.setGoals(m.getToPose()); //enqueue all poses
					
					//start motion planner
					
					//Enqueue the mission

				}
				
				//Save the scenario.
			}
			
			//FIXME How to visualize the generated curves?
		
			@Override
			protected void loop() throws InterruptedException {
				return;
			}
		});
		
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
			if (params.has("/" + node.getName() + "/missions_file")) missionsFile = params.getString("/" + node.getName() + "/missions_file");

		}
		catch (org.ros.exception.ParameterNotFoundException e) {
			System.out.print(ANSI_RED + "== Parameter not found ==");
			System.out.println(ANSI_RESET);
			e.printStackTrace();
		}
	}
}
