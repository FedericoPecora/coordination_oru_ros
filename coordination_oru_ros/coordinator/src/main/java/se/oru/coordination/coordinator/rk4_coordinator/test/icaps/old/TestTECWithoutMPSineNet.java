package se.oru.coordination.coordinator.rk4_coordinator.test.icaps.old;

import java.io.File;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Comparator;
import java.util.Random;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.RVizVisualization;
import se.oru.coordination.coordinator.rk4_coordinator.test.icaps.TrajectoryEnvelopeCoordinatorSimulationICAPS;

@DemoDescription(desc = "Coordination of robots along sine wave paths obtained without the ReedsSheppCarPlanner in opposing directions.")
public class TestTECWithoutMPSineNet {

	public static int usedRobots = 0;
	
	public static boolean inUse(int robotID) {
		if (robotID <= usedRobots) return true;
		return false;
	}
	
	private static PoseSteering[] getSinePath(double period, double magnitude, Pose from, Pose to) {
		if (from.getY() != to.getY()) throw new Error("Can only do straight sine waves ;)");
		ArrayList<Coordinate> coords = new ArrayList<Coordinate>();
		double delta = 0.1;
		double t = 0.0;
		while (t < to.getX()-from.getX()) {
			double value = magnitude*Math.sin(((2*Math.PI)/period)*t);
			Coordinate coord = new Coordinate(t+from.getX(),value+from.getY());
			coords.add(coord);
			t += delta;
		}
		PoseSteering[] pss = new PoseSteering[coords.size()];
		pss[0] = new PoseSteering(coords.get(0).x, coords.get(0).y, Math.atan2(coords.get(1).y-coords.get(0).y, coords.get(1).x-coords.get(0).x),0.0);
		pss[coords.size()-1] = new PoseSteering(coords.get(coords.size()-1).x, coords.get(coords.size()-1).y, Math.atan2(coords.get(coords.size()-1).y-coords.get(coords.size()-2).y, coords.get(coords.size()-1).x-coords.get(coords.size()-2).x),0.0);
		for (int i = 1; i < coords.size()-1; i++) {
			pss[i] = new PoseSteering(coords.get(i).x,coords.get(i).y,Math.atan2(coords.get(i+1).y-coords.get(i).y, coords.get(i+1).x-coords.get(i).x),0.0);
		}		
		return pss;
	}
	
	private static PoseSteering[] invertPath(PoseSteering[] path) {
		PoseSteering[] ret = new PoseSteering[path.length];
		for (int i = 0; i < path.length; i++) {
			ret[i] = path[path.length-i-1];
		}
		return ret;
	}
	
	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 3.0;
		double MAX_VEL = 14.0;
		final int numRobots = Integer.parseInt(args[0]);
		String logFile = System.getProperty("user.home")+File.separator+"icaps_test_3_"+numRobots+"_robots.log";
		String logHeading = "#Rob\t#DrvR\tT\tCS";
		final ArrayList<Integer> robotsInUse = new ArrayList<Integer>();
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		final TrajectoryEnvelopeCoordinatorSimulationICAPS tec = new TrajectoryEnvelopeCoordinatorSimulationICAPS(0, 1000.0, MAX_VEL, MAX_ACCEL, logFile, logHeading, robotsInUse);
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

		Coordinate footprint1 = new Coordinate(-1.0,0.5);
		Coordinate footprint2 = new Coordinate(1.0,0.5);
		Coordinate footprint3 = new Coordinate(1.0,-0.5);
		Coordinate footprint4 = new Coordinate(-1.0,-0.5);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		
		tec.setQuiet(true);

		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
//		String yamlFile = "../maps/map-empty.yaml";
		//JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization(yamlFile);
		RVizVisualization viz = new RVizVisualization();
		tec.setVisualization(viz);

		tec.setUseInternalCriticalPoints(false);
		tec.setYieldIfParking(false);

		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		double deltaY = 3;
		
		String setup = "Sine wave net test with "+numRobots+" robots"+" at "+LocalDateTime.now();
		//tec.writeSetupLog("info", setup);
		
		final int[] robotIDs = new int[numRobots];
		for (int i = 0; i < numRobots; i++) robotIDs[i] = i+1;
		RVizVisualization.writeRVizConfigFile(robotIDs);
		
		for (int index = 0; index < robotIDs.length; index++) {
			int robotID = robotIDs[index];
			double period = 18;
			double mag = deltaY;

			tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(robotID)));
			Pose from = new Pose(0.0,index*deltaY,0.0);
			Pose to = new Pose(5*period,index*deltaY,0.0);
			tec.placeRobot(robotID, from);

			if (index%2 != 0) mag*=(-1);
			PoseSteering[] robotPath = getSinePath(period, mag, from, to);
			PoseSteering[] robotPathInv = invertPath(robotPath);
			Missions.putMission(new Mission(robotID, robotPath));
			Missions.putMission(new Mission(robotID, robotPathInv));
		}

		System.out.println("Added missions " + Missions.getMissions());

		final long durationBeforeIncrement = 20000;
		Thread robotCounter = new Thread() {
			public void run() {
				while (usedRobots < numRobots) {
					robotsInUse.add(robotIDs[usedRobots]);
					usedRobots++;
					System.out.println("ROBOTS IN USE: " + usedRobots);
					try { Thread.sleep(durationBeforeIncrement); }
					catch (InterruptedException e) { e.printStackTrace(); }
				}
				System.out.println("TEST FINISHED!");
			}
		};
		robotCounter.start();
		
		//Start the thread that revises precedences at every period
		tec.startInference();
		
		//Start a mission dispatching thread for each robot, which will run forever
		for (final int robotID : robotIDs) {
			//For each robot, create a thread that dispatches the "next" mission when the robot is free 
			Thread t = new Thread() {
				int iteration = 0;
				@Override
				public void run() {
					this.setPriority(Thread.MAX_PRIORITY);
					while (usedRobots <= numRobots) {
						if (inUse(robotID)) {
							//Mission to dispatch alternates between (rip -> desti) and (desti -> rip)
							Mission m = Missions.getMission(robotID, iteration%2);
							synchronized(tec) {
								//addMission returns true iff the robot was free to accept a new mission
								if (tec.addMissions(m)) {
									//tec.writeSetupLog("progress", "Robot "+robotID+" has completed " +iteration+" missions.");
									iteration++;
								}
							}
						}
						//Sleep for a little (2 sec)
						try { Thread.sleep(2000); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}
				}
			};
			//Start the thread!
			t.start();
		}
	}

}
