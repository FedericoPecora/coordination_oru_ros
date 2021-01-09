package se.oru.coordination.coordinator.rk4_coordinator.test;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Comparator;
import java.util.Random;

import javax.imageio.ImageIO;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

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
import se.oru.coordination.coordination_oru.util.RVizVisualization;

@DemoDescription(desc = "Coordination of 4 robots along wave-like paths obtained with the ReedsSheppCarPlanner in opposing directions.")
public class TestTrajectoryEnvelopeCoordinatorWithMotionPlanner4 {

	public static int MIN_DELAY = 500;
	public static int MAX_DELAY = 0;

	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 3.0;
		double MAX_VEL = 14.0;
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(3000, 1000.0, MAX_VEL,MAX_ACCEL);
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

		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
//		String yamlFile = "../maps/map-empty.yaml";
		//JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization(yamlFile);
		RVizVisualization viz = new RVizVisualization();
		tec.setVisualization(viz);

		tec.setUseInternalCriticalPoints(false);
		tec.setYieldIfParking(false);

		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		//Instantiate a simple motion planner
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
//		rsp.setMap(yamlFile);
		rsp.setRadius(0.2);
		rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.5);
		double deltaY = 3;
		double height = deltaY/2;
		double mapHeight = 100;

//		try {
//			BufferedImage img = ImageIO.read(new File(mapFile));
//			mapHeight = img.getHeight()*res*0.9;
//		}
//		catch (IOException e) { e.printStackTrace(); }

		int numRobots = 15;
		int[] robotIDs = new int[numRobots];
		for (int i = 0; i < numRobots; i++) robotIDs[i] = i+1;
		RVizVisualization.writeRVizConfigFile(robotIDs);
		
		for (int index = 0; index < robotIDs.length; index++) {
			int robotID = robotIDs[index];
			//You probably also want to provide a non-trivial forward model
			//(the default assumes that robots can always stop)
			tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
			ArrayList<Pose> posesRobot = new ArrayList<Pose>();
			//if (index%2==0) {
			if (robotID%2==0) {
				posesRobot.add(new Pose(2.0,mapHeight-deltaY-height*index,0.0));
				posesRobot.add(new Pose(10.0,mapHeight-height*index,0.0));
				posesRobot.add(new Pose(18.0,mapHeight-deltaY-height*index,0.0));
				posesRobot.add(new Pose(26.0,mapHeight-height*index,0.0));
				posesRobot.add(new Pose(34.0,mapHeight-deltaY-height*index,0.0));
				posesRobot.add(new Pose(42.0,mapHeight-height*index,0.0));
				posesRobot.add(new Pose(50.0,mapHeight-deltaY-height*index,0.0));
			}
			else {
				posesRobot.add(new Pose(50.0,mapHeight-height*(index-1),Math.PI));
				posesRobot.add(new Pose(42.0,mapHeight-deltaY-height*(index-1),Math.PI));
				posesRobot.add(new Pose(34.0,mapHeight-height*(index-1),Math.PI));
				posesRobot.add(new Pose(26.0,mapHeight-deltaY-height*(index-1),Math.PI));
				posesRobot.add(new Pose(18.0,mapHeight-height*(index-1),Math.PI));
				posesRobot.add(new Pose(10.0,mapHeight-deltaY-height*(index-1),Math.PI));
				posesRobot.add(new Pose(2.0,mapHeight-height*(index-1),Math.PI));
			}
			tec.placeRobot(robotID, posesRobot.get(0));
			
			//Path planner to use for re-planning if needed
			tec.setMotionPlanner(robotID, rsp);
			
			rsp.setStart(posesRobot.get(0));
			rsp.setGoals(posesRobot.subList(1, posesRobot.size()).toArray(new Pose[posesRobot.size()-1]));
			rsp.clearObstacles();
			if (!rsp.plan()) throw new Error ("No path along goals " + posesRobot);			
			PoseSteering[] robotPath = rsp.getPath();
			PoseSteering[] robotPathInv = rsp.getPathInv();
			
			Missions.putMission(new Mission(robotID, robotPath));
			Missions.putMission(new Mission(robotID, robotPathInv));
		}

		System.out.println("Added missions " + Missions.getMissions());
		
		//Start the thread that revises precedences at every period
		tec.startInference();

		//Start dispatching threads for each robot, each of which
		//dispatches the next mission as soon as the robot is idle
		Missions.startMissionDispatchers(tec, robotIDs);

	}

}
