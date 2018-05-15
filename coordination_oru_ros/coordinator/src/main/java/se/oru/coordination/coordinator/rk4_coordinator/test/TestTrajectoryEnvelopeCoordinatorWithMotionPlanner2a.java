package se.oru.coordination.coordinator.rk4_coordinator.test;

import java.util.Calendar;
import java.util.Comparator;

import org.metacsp.multi.spatioTemporal.paths.Pose;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

@DemoDescription(desc = "Coordination on paths obtained from the ReedsSheppCarPlanner for two robots navigating in the same direction.")
public class TestTrajectoryEnvelopeCoordinatorWithMotionPlanner2a {

	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 4.0;
		double MAX_VEL = 14.0;
		int CONTROL_PERIOD = 500;
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(CONTROL_PERIOD,1000.0,MAX_VEL,MAX_ACCEL);
//		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
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
		tec.setYieldIfParking(false);
		tec.setBreakDeadlocks(true);
		
		Missions.loadLocationAndPathData("../missions/output/locations_and_paths.txt");

		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		int[] robotIDs = new int[] {1,2,3,4,5,6};
		//int[] robotIDs = new int[] {1,2};
		for (int robotID : robotIDs) {
			tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTrackingPeriod(), tec.getTemporalResolution()));
			String startLocName = "START_"+(robotID-1);
			Pose startLoc = Missions.getLocation(startLocName);
			tec.placeRobot(robotID, startLoc);
			System.out.println("Placed Robot" + robotID + " in " + startLocName);
			
			if (robotID%2 == 0) {
				Mission m = new Mission(robotID, Missions.loadKnownPath(startLocName, "G4a2"), startLocName, "G4a2", Missions.getLocation(startLocName), Missions.getLocation("G4a2"));
				Missions.putMission(m);
				Mission m1 = new Mission(robotID, Missions.loadKnownPath("G4a2","G1"), "G4a2", "G1", Missions.getLocation("G4a2"), Missions.getLocation("G1"));
				Missions.putMission(m1);
				Mission m2 = new Mission(robotID, Missions.loadKnownPath("G1","G4a2"), "G1", "G4a2", Missions.getLocation("G1"), Missions.getLocation("G4a2"));
				Missions.putMission(m2);
			}
			else {
				Mission m = new Mission(robotID, Missions.loadKnownPath(startLocName, "G4b"), startLocName, "G4b", Missions.getLocation(startLocName), Missions.getLocation("G4b"));
				Missions.putMission(m);
				Mission m1 = new Mission(robotID, Missions.loadKnownPath("G4b","G1"), "G4b", "G1", Missions.getLocation("G4b"), Missions.getLocation("G1"));
				Missions.putMission(m1);
				Mission m2 = new Mission(robotID, Missions.loadKnownPath("G1","G4b"), "G1", "G4b", Missions.getLocation("G1"), Missions.getLocation("G4b"));
				Missions.putMission(m2);
			}
		}

		System.out.println("Added missions " + Missions.getMissions());

		//Dispatch first missions (popping them, so they will disappear from the queue)
		for (int robotID : robotIDs) {
			Mission m = Missions.popMission(robotID);
			//addMission returns true iff the robot was free to accept a new mission
			tec.addMissions(m);
			tec.computeCriticalSections();
			tec.startTrackingAddedMissions();
		}

		//Start a mission dispatching thread for each robot, which will run forever
		for (int i = 0; i < robotIDs.length; i++) {
			final int robotID = robotIDs[i];
			//For each robot, create a thread that dispatches the "next" mission when the robot is free 
			Thread t = new Thread() {
				@Override
				public void run() {
					int sequenceNumber = 0;
					String lastDestination = "";
					if (robotID%2 == 0) lastDestination = "G4a2";
					else lastDestination = "G4b";
					long startTime = Calendar.getInstance().getTimeInMillis();
					while (true) {
						//Mission to dispatch alternates between (rip -> desti) and (desti -> rip)
						synchronized(tec) {
							if (tec.isFree(robotID)) {
								long elapsed = Calendar.getInstance().getTimeInMillis()-startTime;
								System.out.println("Time to reach " + lastDestination + " (Robot" + robotID + "): " + elapsed);
								startTime = Calendar.getInstance().getTimeInMillis();
								Mission m = Missions.getMission(robotID,sequenceNumber);
								tec.addMissions(m);
								tec.computeCriticalSections();
								tec.startTrackingAddedMissions();
								sequenceNumber = (sequenceNumber+1)%Missions.getMissions(robotID).size();
								lastDestination = m.getToLocation();
							}
						}
						//Sleep for a little (2 sec)
						try { Thread.sleep(100); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}
				}
			};
			//Start the thread!
			t.start();
		}

	}

}
