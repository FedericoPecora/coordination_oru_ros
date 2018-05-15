package se.oru.coordination.coordinator.rk4_coordinator.test;

import java.io.File;
import java.util.Comparator;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

@DemoDescription(desc = "Coordination on paths obtained from the ReedsSheppCarPlanner for two robots navigating in the same direction.")
public class TestTrajectoryEnvelopeCoordinatorWithMotionPlanner2 {

	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 1.0;
		double MAX_VEL = 4.0;
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				CriticalSection cs = o1.getCriticalSection();
				RobotReport robotReport1 = o1.getTrajectoryEnvelopeTracker().getRobotReport();
				RobotReport robotReport2 = o2.getTrajectoryEnvelopeTracker().getRobotReport();
				return ((cs.getTe1Start()-robotReport1.getPathIndex())-(cs.getTe2Start()-robotReport2.getPathIndex()));
			}
		});
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				return (o2.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID()-o1.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID());
			}
		});

		//You probably also want to provide a non-trivial forward model
		//(the default assumes that robots can always stop)
		tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTrackingPeriod(), tec.getTemporalResolution()));
		tec.setForwardModel(2, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTrackingPeriod(), tec.getTemporalResolution()));

		Coordinate footprint1 = new Coordinate(-1.0,0.5);
		Coordinate footprint2 = new Coordinate(1.0,0.5);
		Coordinate footprint3 = new Coordinate(1.0,-0.5);
		Coordinate footprint4 = new Coordinate(-1.0,-0.5);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);

		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		String yamlFile = "../maps/map-partial-1.yaml";
		JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization(yamlFile);
		tec.setVisualization(viz);

		tec.setUseInternalCriticalPoints(false);
		
		Missions.loadLocationAndPathData("../missions/icaps_locations_and_paths.txt");

		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		//Instantiate a simple motion planner
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setMapFilename("maps"+File.separator+Missions.getProperty("image", yamlFile));
		double res = Double.parseDouble(Missions.getProperty("resolution", yamlFile));
		rsp.setMapResolution(res);
		rsp.setRadius(0.1);
		rsp.setFootprint(tec.getDefaultFootprint());
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.3);

		int[] robotIDs = new int[] {1,2,3,4,5,6};
		for (int robotID : robotIDs) {
			String startLocName = "START_"+(robotID-1);
			Pose startLoc = Missions.getLocation(startLocName);
			tec.placeRobot(robotID, startLoc);
			System.out.println("Placed Robot" + robotID + " in " + startLocName);

			String fromName = startLocName;
			Pose from = startLoc;
			String toName = "G1";
			Pose to = Missions.getLocation(toName);
			rsp.setStart(from);
			rsp.setGoals(to);
			rsp.plan();
			Mission m1 = new Mission(robotID, rsp.getPath(), fromName, toName, from, to);
			Missions.putMission(m1);
			
			fromName = toName;
			from = to;
			toName = "G2";
			to = Missions.getLocation(toName);
			rsp.setStart(from);
			rsp.setGoals(to);
			rsp.plan();
			Mission m2 = new Mission(robotID, rsp.getPath(), fromName, toName, from, to);
			Missions.putMission(m2);
			
			fromName = toName;
			from = to;
			toName = "G3";
			to = Missions.getLocation(toName);
			rsp.setStart(from);
			rsp.setGoals(to);
			rsp.plan();
			Mission m3 = new Mission(robotID, rsp.getPath(), fromName, toName, from, to);
			Missions.putMission(m3);
			
			if (robotID%2 == 0) {
				fromName = toName;
				from = to;
				String toName1 = "G4a1";
				Pose to1 = Missions.getLocation(toName1);
				toName = "G4a2";
				to = Missions.getLocation(toName);
				rsp.setStart(from);
				rsp.setGoals(to1,to);
				rsp.plan();
				Mission m4 = new Mission(robotID, rsp.getPath(), fromName, toName, from, to);
				Missions.putMission(m4);
			}
			else {
				fromName = toName;
				from = to;
				toName = "G4b";
				to = Missions.getLocation(toName);
				rsp.setStart(from);
				rsp.setGoals(to);
				rsp.plan();
				Mission m4 = new Mission(robotID, rsp.getPath(), fromName, toName, from, to);
				Missions.putMission(m4);				
			}
			
			fromName = toName;
			from = to;
			toName = "G5";
			to = Missions.getLocation(toName);
			rsp.setStart(from);
			rsp.setGoals(to);
			rsp.plan();
			Mission m5 = new Mission(robotID, rsp.getPath(), fromName, toName, from, to);
			Missions.putMission(m5);
			
			fromName = toName;
			from = to;
			toName = "G1";
			to = Missions.getLocation(toName);
			rsp.setStart(from);
			rsp.setGoals(to);
			rsp.plan();
			Mission m6 = new Mission(robotID, rsp.getPath(), fromName, toName, from, to);
			Missions.putMission(m6);

		}

		System.out.println("Added missions " + Missions.getMissions());

		//Dispatch first missions (popping them, so they will disappear from the queue)
		for (int robotID : robotIDs) {
			Mission m = Missions.popMission(robotID);
			synchronized(tec) {
				//addMission returns true iff the robot was free to accept a new mission
				if (tec.addMissions(m)) {
					tec.computeCriticalSections();
					tec.startTrackingAddedMissions();
				}
			}
		}

		//Start a mission dispatching thread for each robot, which will run forever
		for (int i = 1; i <= 2; i++) {
			final int robotID = i;
			//For each robot, create a thread that dispatches the "next" mission when the robot is free 
			Thread t = new Thread() {
				@Override
				public void run() {
					while (true) {
						//Mission to dispatch alternates between (rip -> desti) and (desti -> rip)
						Mission m = Missions.popMission(robotID);
						synchronized(tec) {
							//addMission returns true iff the robot was free to accept a new mission
							if (tec.addMissions(m)) {
								tec.computeCriticalSections();
								tec.startTrackingAddedMissions();
							}
						}
						Missions.putMission(m);
						//Sleep for a little (2 sec)
						try { Thread.sleep(1000); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}
				}
			};
			//Start the thread!
			t.start();
		}

	}

}
