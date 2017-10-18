package se.oru.coordination.coordinator.ros_coordinator.simulation2D;

import java.io.File;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.Random;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.util.AffineTransformation;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordinator.util.RVizVisualization;

@DemoDescription(desc = "Simple test.")
public class TestTrajectoryEnvelopeCoordinatorDellrudLoop {
	
	private static ReedsSheppCarPlanner rsp = null;
	
	private static String[] depotLocations = new String[] {
			"pre_in",
			"in",
			"pull_in_x",
			"maint_x",
			"fuel_x",
			"park_x",
			"pre_out",
			"out"
	};
	
	private static HashMap<String,Integer> numberOfPlacesInLocation = new HashMap<String, Integer>();
	
	static {
		numberOfPlacesInLocation.put("pull_in_x", 3);
		numberOfPlacesInLocation.put("fuel_x", 3);
		numberOfPlacesInLocation.put("maint_x", 3);
		numberOfPlacesInLocation.put("park_x", 6);
	}

	//door 1: (26.0891, 39.4392, -1.5624)
	//door 2: (267.0891, 37.2392, 1.5596)
	private static Pose doorPose1 = new Pose(26.0891, 39.4392, -1.5624);
	private static Pose doorPose2 = new Pose(267.0891, 37.2392, 1.5596);

	//Klasse footprint
	static Coordinate[] klasseFootprint = new Coordinate[] {
			new Coordinate(-3.395,1.275),
			new Coordinate(-3.395,-1.275),
			new Coordinate(9.795,-1.275),
			new Coordinate(9.795,1.275)
	};

	private static HashMap<Integer,String> currentBusDestinations = new HashMap<Integer, String>();

	private static void setupRRTMotionPlanner(String map, Coordinate ... footprint) {
		//Instantiate a simple motion planner
		rsp = new ReedsSheppCarPlanner();
		rsp.setMapFilename("/home/fpa/gitroot.gitlab/iqmobility/maps"+File.separator+"dellrud-depot-scania"+File.separator+Missions.getProperty("image", map));
		double res = Double.parseDouble(Missions.getProperty("resolution", map));
		rsp.setMapResolution(res);
		rsp.setRadius(0.2);
		rsp.setFootprint(footprint);
		rsp.setTurningRadius(10.0);
		rsp.setDistanceBetweenPathPoints(0.5);		
	}
	
	private static Geometry[] makeCurrentObstacles(TrajectoryEnvelopeCoordinator tec, Coordinate[] footprint, String source, String destination, ArrayList<Integer> robotIDs) {
		ArrayList<Geometry> ret = new ArrayList<Geometry>();
		for (int robotID : robotIDs) {
			RobotReport rr = tec.getRobotReport(robotID);
			int pathIndex = rr.getPathIndex();
			Pose obstaclePose = null;
			if (pathIndex != -1 && !currentBusDestinations.get(robotID).equals(source) && !currentBusDestinations.get(robotID).equals(destination)) {
				int lastPoseIndex = tec.getCurrentTrajectoryEnvelope(robotID).getTrajectory().getPose().length-1;
				obstaclePose = tec.getCurrentTrajectoryEnvelope(robotID).getTrajectory().getPose()[lastPoseIndex];
				GeometryFactory gf = new GeometryFactory();
				Coordinate[] newFoot = new Coordinate[footprint.length+1];
				for (int j = 0; j < footprint.length; j++) {
					newFoot[j] = footprint[j];
				}
				newFoot[footprint.length] = footprint[0];
				Geometry obstacle = gf.createPolygon(newFoot);
				AffineTransformation at = new AffineTransformation();
				at.rotate(obstaclePose.getTheta());
				at.translate(obstaclePose.getX(), obstaclePose.getY());
				obstacle = at.transform(obstacle);
				ret.add(obstacle);
			}
		}
		return ret.toArray(new Geometry[ret.size()]);
	}
	
	private static Geometry[] makeCurrentObstacles(Coordinate[] footprint, Pose ... obstaclePoses) {
		ArrayList<Geometry> ret = new ArrayList<Geometry>();
		for (Pose p : obstaclePoses) {
			GeometryFactory gf = new GeometryFactory();
			Coordinate[] newFoot = new Coordinate[footprint.length+1];
			for (int j = 0; j < footprint.length; j++) {
				newFoot[j] = footprint[j];
			}
			newFoot[footprint.length] = footprint[0];
			Geometry obstacle = gf.createPolygon(newFoot);
			AffineTransformation at = new AffineTransformation();
			at.rotate(p.getTheta());
			at.translate(p.getX(), p.getY());
			obstacle = at.transform(obstacle);
			ret.add(obstacle);
		}
		return ret.toArray(new Geometry[ret.size()]);
	}

	public static PoseSteering[] computePathWithRRTPlanner(Pose from, Pose to, Geometry ... obstacles) {
		synchronized(rsp) {
			rsp.clearObstacles();
			if (obstacles != null && obstacles.length > 0) {
				rsp.addObstacles(obstacles);
			}
			
			rsp.setStart(from);
			rsp.setGoals(to);
			
			if (!rsp.plan()) return null;
			return rsp.getPath();
		}
	}
	
	public static String getNextLocation(int robotID) {
		Random rand = new Random(Calendar.getInstance().getTimeInMillis());
		for (int i = 0; i < depotLocations.length; i++) {
			if (currentBusDestinations.get(robotID).startsWith(depotLocations[i].replaceAll("_x", ""))) {
				String toReturn = depotLocations[(i+1)%depotLocations.length];
				if (toReturn.startsWith("maint") && robotID%2 != 0) toReturn = depotLocations[(i+2)%depotLocations.length];
				if (toReturn.contains("_x")) {
					int positionNumber = rand.nextInt(numberOfPlacesInLocation.get(toReturn))+1;
					toReturn = toReturn.replaceAll("_x", "_"+positionNumber);
				}
				return toReturn;
			}
		}
		return null;
	}
	
	public static PoseSteering[] computePathWithRRTPlanner(String from, String to, Geometry ... obstacles) {
		Pose start = Missions.getLocation(from);
		Pose goal = Missions.getLocation(to);
		return computePathWithRRTPlanner(start, goal, obstacles);
	}

	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 2.0;
		double MAX_VEL = 15.0;
		int CONTROL_PERIOD = 1000;
		double TEMPORAL_RESOLUTION = 1000.0;
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(CONTROL_PERIOD, TEMPORAL_RESOLUTION, MAX_VEL,MAX_ACCEL);

		//Order by distance to critical section (closest to critical section start goes first if possible)
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				CriticalSection cs = o1.getCriticalSection();
				RobotReport robotReport1 = o1.getTrajectoryEnvelopeTracker().getRobotReport();
				RobotReport robotReport2 = o2.getTrajectoryEnvelopeTracker().getRobotReport();
				return ((cs.getTe1Start()-robotReport1.getPathIndex())-(cs.getTe2Start()-robotReport2.getPathIndex()));
			}
		});
		
		//Order by robot ID (lower ID goes first if possible)
//		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
//			@Override
//			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
//				return (o1.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID()-o2.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID());
//			}
//		});
		
		//Can set a refresh rate in GUI (lower than default, in msec), this may help lower the computational overhead
		//tec.setGUIUpdatePeriod(200);
		
		//Robots that will park in a critical section should not yield to other robots
		tec.setYieldIfParking(true);
		//Completely overlapping paths should not lead to failure
		tec.setCheckEscapePoses(false);
		//Do not use internal critical points to slow down at cusps
		tec.setUseInternalCriticalPoints(false);
		//Do not attempt to break deadlocks
		//tec.setBreakDeadlocks(false);
		//tec.setGUIUpdatePeriod(300);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);

		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		final String yamlFile = "/home/fpa/gitroot.gitlab/iqmobility/maps/dellrud-depot-scania/dellrud-depot-scania.yaml";
		//final JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization(tec,yamlFile);
		RVizVisualization viz = new RVizVisualization();
		//viz.setMapFileName(yamlFile, "/home/fpa/gitroot.gitlab/iqmobility/maps/dellrud-depot-scania");
		tec.setVisualization(viz);
		
		Missions.loadLocationAndPathData("/home/fpa/gitroot.gitlab/iqmobility/paths/poses.txt");
				
		tec.setDefaultFootprint(klasseFootprint);
		setupRRTMotionPlanner(yamlFile, klasseFootprint);
		
		//Randomly put buses in locations
		Random rand = new Random(Calendar.getInstance().getTimeInMillis());
		int[] robots = new int[] {1,2,3,4,5,6};
		for (int robotID : robots) {
			tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, CONTROL_PERIOD, TEMPORAL_RESOLUTION));
			String nextLocationName = null;
			while (true) {
				int nextLocation = rand.nextInt(depotLocations.length);
				nextLocationName = depotLocations[nextLocation];
				if (nextLocationName.contains("_x")) {
					int positionNumber = rand.nextInt(numberOfPlacesInLocation.get(nextLocationName))+1;
					nextLocationName = nextLocationName.replaceAll("_x", "_"+positionNumber);
				}
				boolean alreadyUsed = false;
				for (Entry<Integer,String> entry : currentBusDestinations.entrySet()) {
					if (entry.getValue().equals(nextLocationName)) {
						alreadyUsed = true;
					}
				}
				if (!alreadyUsed) break;
			}
			tec.placeRobot(robotID, Missions.getLocation(nextLocationName));
			currentBusDestinations.put(robotID, nextLocationName);
		}
		
		//Send buses to next locations when they are free
		while (true) {
			for (int robotID : robots) {
				if (tec.isFree(robotID)) {
					ArrayList<Integer> otherRobots = new ArrayList<Integer>();
					for (Integer otherRobot : robots) if (otherRobot != robotID) otherRobots.add(otherRobot);
					String currentLocation = currentBusDestinations.get(robotID);
					String nextLocation = getNextLocation(robotID);
					PoseSteering[] path = null;
					if (nextLocation.equals("out") || nextLocation.equals("in")) {
						Geometry[] obstacles = makeCurrentObstacles(tec, klasseFootprint, currentLocation, nextLocation, otherRobots);
						path = computePathWithRRTPlanner(Missions.getLocation(currentLocation), Missions.getLocation(nextLocation), obstacles);
						if (path == null) System.out.println("NO PATH FOUND W/O DOORS BETWEEN " + currentLocation + " and " + nextLocation);
					}
					else {
						Geometry[] obstacles = makeCurrentObstacles(tec, klasseFootprint, currentLocation, nextLocation, otherRobots);
						Geometry[] doors = makeCurrentObstacles(klasseFootprint, doorPose1, doorPose2);
						ArrayList<Geometry> allObstacles = new ArrayList<Geometry>();
						for (Geometry obs : obstacles) allObstacles.add(obs);
						for (Geometry obs : doors) allObstacles.add(obs);
						path = computePathWithRRTPlanner(Missions.getLocation(currentLocation), Missions.getLocation(nextLocation), allObstacles.toArray(new Geometry[allObstacles.size()]));
						if (path == null) System.out.println("NO PATH FOUND WITH DOORS BETWEEN " + currentLocation + " and " + nextLocation);
					}
					Mission m = new Mission(robotID, path);
					currentBusDestinations.put(robotID, nextLocation);
					tec.addMissions(m);
					tec.computeCriticalSections();
					tec.startTrackingAddedMissions();
				}
			}
			Thread.sleep(500);
		}
						
	}
	
}
