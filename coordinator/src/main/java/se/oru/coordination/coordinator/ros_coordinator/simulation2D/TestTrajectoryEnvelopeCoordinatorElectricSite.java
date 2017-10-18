package se.oru.coordination.coordinator.ros_coordinator.simulation2D;

import java.io.File;
import java.util.Comparator;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.MissionDispatchingCallback;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordinator.util.RVizVisualization;

@DemoDescription(desc = "Simple test showing the use of pre-planned paths stored in files.")
public class TestTrajectoryEnvelopeCoordinatorElectricSite {
	
	private static ReedsSheppCarPlanner rsp = null;

	private static void setupRRTMotionPlanner(String map, Coordinate ... footprint) {
		//Instantiate a simple motion planner
		rsp = new ReedsSheppCarPlanner();
		rsp.setMapFilename("/home/fpa/gitroot.gitlab/volvo_ce/coordination_oru_vce/maps"+File.separator+Missions.getProperty("image", map));
		double res = Double.parseDouble(Missions.getProperty("resolution", map));
		rsp.setMapResolution(res);
		rsp.setRadius(0.2);
		rsp.setFootprint(footprint);
		rsp.setTurningRadius(2.0);
		rsp.setDistanceBetweenPathPoints(0.5);		
	}
	
	public static PoseSteering[] computePathWithRRTPlanner(Pose from, Pose to, Geometry ... obstacles) {
		synchronized(rsp) {
			if (obstacles != null && obstacles.length > 0) {
				rsp.clearObstacles();
				rsp.addObstacles(obstacles);
			}
			
			rsp.setStart(from);
			rsp.setGoals(to);
			
			if (!rsp.plan()) return null;
			return rsp.getPath();
		}
	}
	
	public static PoseSteering[] computePathWithRRTPlanner(String from, String to, Geometry ... obstacles) {
		Pose start = Missions.getLocation(from);
		Pose goal = Missions.getLocation(to);
		return computePathWithRRTPlanner(start, goal, obstacles);
	}

	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 3.0;
		double MAX_VEL = 10.0;
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);

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
		tec.setYieldIfParking(false);
		//Completely overlapping paths should not lead to failure
		tec.setCheckEscapePoses(false);
		//Do not use internal critical points to slow down at cusps
		tec.setUseInternalCriticalPoints(false);
		//Do not attempt to break deadlocks
		//tec.setBreakDeadlocks(false);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);

		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		final String yamlFile = "/home/fpa/gitroot.gitlab/volvo_ce/coordination_oru_vce/maps/elsite_1m.yaml";
		//final JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization(yamlFile);
		RVizVisualization viz = new RVizVisualization();
		viz.setMapFileName(yamlFile, "/home/fpa/gitroot.gitlab/volvo_ce/coordination_oru_vce/maps");
		tec.setVisualization(viz);
		
		setupRRTMotionPlanner(yamlFile, tec.getDefaultFootprint());

		//Load data file with locations and pointers to files containing paths between locations
		Missions.loadLocationAndPathData("/home/fpa/gitroot.gitlab/volvo_ce/coordination_oru_vce/paths/elsite_data_smooth_paths_no_ip.txt");
		String prefix = "/home/fpa/gitroot.gitlab/volvo_ce/coordination_oru_vce/paths/";

		//Make a WheelLoaderSimulator that cycles thru WL goals 
		Coordinate[] wlFootprint = new Coordinate[] {
				new Coordinate(1.0,-1.0),
				new Coordinate(1.0,1.0),
				new Coordinate(-1.0,1.0),
				new Coordinate(-1.0,-1.0),
				new Coordinate(1.0,-1.0)
		};
		Pose[] wlPoses = new Pose[] {
				new Pose(147.0, 377.0, 0.0),
				new Pose(155.0, 370.0, 0.0),
				new Pose(163.0, 363.0, 0.0)
		};
		final WheelLoaderSimulator wlSim = new WheelLoaderSimulator(2000, wlFootprint, wlPoses) {
			@Override
			public void callback() {
				//if (viz.getPanel() != null) viz.getPanel().addGeometry("wl", getCurrentGeometryOfWheelLoader());				
			}
		};
		wlSim.start();

		int[] robots = new int[] {1,2,3,4,5,6,7,8};
		//int[] robots = new int[] {1,2,3,4,5,6};
		//int[] robots = new int[] {1,2,3,4};
		//int[] robots = new int[] {1,2,3};
		for (int robotID : robots) {

			//You probably also want to provide a non-trivial forward model
			//(the default assumes that robots can always stop)
			tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTrackingPeriod(), tec.getTemporalResolution(), 10));
			
			//Place robots in their initial locations (looked up in the data file that was loaded above)
			// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
			// -- each trajectory envelope has a path of one pose (the pose of the location)
			// -- each trajectory envelope is the footprint of the corresponding robot in that pose
			tec.placeRobot(robotID, Missions.getLocation("r" + robotID + "p"));
			
			//Create missions for visiting all destinations in the Electric Site 
			int[] chargers = new int[] {1,2};
			String location = "r"+robotID+"p";
			String charger = "c" + chargers[((robotID+1)%2)];
			Mission p2ch = new Mission(robotID,prefix+Missions.getPathFile(location, charger),Missions.getLocation(location),Missions.getLocation(charger));
			Missions.putMission(p2ch);
			Mission ch2pcr = new Mission(robotID,prefix+Missions.getPathFile(charger, "pcr"), Missions.getLocation(charger),Missions.getLocation("pcr"));
			Missions.putMission(ch2pcr);
			//Mission pcr2wl = new Mission(robotID,prefix+Missions.getPathFile("pcr", "wl"),Missions.getLocation("pcr"),Missions.getLocation("wl"));
			//Missions.putMission(pcr2wl);
			Mission pcr2prewl = new Mission(robotID,prefix+Missions.getPathFile("pcr", "prewl"),Missions.getLocation("pcr"),Missions.getLocation("prewl"));
			Missions.putMission(pcr2prewl);
			//Mission wl2scr = new Mission(robotID,prefix+Missions.getPathFile("wl", "scr"),Missions.getLocation("wl"),Missions.getLocation("scr"));
			//Missions.putMission(wl2scr);
			Mission prewl2wl = new Mission(robotID, "prewl",  "wl", Missions.getLocation("prewl"), null);
			Missions.putMission(prewl2wl);
			Mission wl2postwl = new Mission(robotID, "wl",  "postwl", null, Missions.getLocation("postwl"));
			Missions.putMission(wl2postwl);
			Mission postwl2scr = new Mission(robotID,prefix+Missions.getPathFile("postwl", "scr"),Missions.getLocation("postwl"),Missions.getLocation("scr"));
			Missions.putMission(postwl2scr);
			Mission scr2ch = new Mission(robotID,prefix+Missions.getPathFile("scr", charger),Missions.getLocation("scr"),Missions.getLocation(charger));
			Missions.putMission(scr2ch);
			
			Missions.concatenateMissions(pcr2prewl,prewl2wl);
			Missions.concatenateMissions(wl2postwl,postwl2scr);
			
			Missions.addMissionDispatchingCallback(robotID, new MissionDispatchingCallback() {
				private Pose myWLPose = null;
				@Override
				public void beforeMissionDispatch(Mission m) {
					if (m.getToPose() == null) {
						myWLPose = wlSim.getCurrentPoseOfWheelLoader();
						PoseSteering[] path = computePathWithRRTPlanner(m.getFromPose(), myWLPose, new Geometry[] {});
						m.setPath(path);
					}
					else if (m.getFromPose() == null) {
						PoseSteering[] path = computePathWithRRTPlanner(myWLPose, m.getToPose(), new Geometry[] {});
						m.setPath(path);
					}
				}
				@Override
				public void afterMissionDispatch(Mission m) { }
			});

		}

		System.out.println("Added missions " + Missions.getMissions());
		
		Missions.startMissionDispatchers(tec, robots);
	}

}
