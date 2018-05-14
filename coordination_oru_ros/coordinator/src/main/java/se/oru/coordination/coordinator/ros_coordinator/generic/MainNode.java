/*
 * Copyright (C) 2014 Federico Pecora.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package se.oru.coordination.coordinator.ros_coordinator.generic;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Scanner;
import java.util.concurrent.TimeUnit;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Quaternion;
import org.ros.concurrent.CancellableLoop;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Subscriber;

import com.vividsolutions.jts.geom.Coordinate;

import cern.colt.Arrays;
import orunav_msgs.ComputeTask;
import orunav_msgs.ComputeTaskRequest;
import orunav_msgs.ComputeTaskResponse;
import orunav_msgs.CoordinatorTime;
import orunav_msgs.CoordinatorTimeVec;
import orunav_msgs.DeltaTVec;
import orunav_msgs.ExecuteTask;
import orunav_msgs.ExecuteTaskRequest;
import orunav_msgs.ExecuteTaskResponse;
import orunav_msgs.Operation;
import orunav_msgs.RobotTarget;
import orunav_msgs.Task;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.RVizVisualization;
import se.oru.coordination.coordinator.ros_coordinator.TrajectoryEnvelopeCoordinatorROS;

public class MainNode extends AbstractNodeMain {

	private List<Integer> robotIDs = null;
	private HashMap<Integer,Pose> initialLocations = new HashMap<Integer,Pose>();
	private ConnectedNode node = null;
	private HashMap<Integer,Integer> robotIDstoGoalIDs = new HashMap<Integer,Integer>();
	private TrajectoryEnvelopeCoordinatorROS tec = null;
	private Coordinate[] footprintCoords = null;
	private int CONTROL_PERIOD = 1000;
	private double TEMPORAL_RESOLUTION = 1000.0;
	private double MAX_ACCEL = 1.0;
	private double MAX_VEL = 4.0;
	private String locationsFile = null;
	private String goalSequenceFile = null;
	private HashMap<Integer,Boolean> computing = new HashMap<Integer,Boolean>();
	private HashMap<Integer,Integer> missionNumber = new HashMap<Integer,Integer>();
    
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
						RobotReport robotReport1 = o1.getTrajectoryEnvelopeTracker().getRobotReport();
						RobotReport robotReport2 = o2.getTrajectoryEnvelopeTracker().getRobotReport();
						return ((cs.getTe1Start()-robotReport1.getPathIndex())-(cs.getTe2Start()-robotReport2.getPathIndex()));
					}
				});
				
				//Need to setup infrastructure that maintains the representation
				tec.setupSolver(origin, origin+100000000L);
				tec.setYieldIfParking(false);
				
				//Setup a simple GUI (null means empty map, otherwise provide yaml file)
				//final JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
				final RVizVisualization viz = new RVizVisualization(node);
				tec.setVisualization(viz);
				
				//Set the footprint of the robots
				tec.setDefaultFootprint(footprintCoords);
				if (locationsFile != null) Missions.loadLocationAndPathData(locationsFile);
				if (goalSequenceFile != null) readGoalSequenceFile();
				
				//Sleep to allow loading of motion prims
				try {
					Thread.sleep(10000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				
				for (final int robotID : robotIDs) {
					
					computing.put(robotID, false);
					missionNumber.put(robotID, 0);
					
					//Set the forward dynamic model for the robot so the coordinator
					//can estimate whether the robot can stop
					tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, CONTROL_PERIOD, TEMPORAL_RESOLUTION));
					
					//Get all initial locations of robots (this is done once)
					Subscriber<orunav_msgs.RobotReport> subscriberInit = node.newSubscriber("robot"+robotID+"/report", orunav_msgs.RobotReport._TYPE);
					subscriberInit.addMessageListener(new MessageListener<orunav_msgs.RobotReport>() {
						@Override
						public void onNewMessage(orunav_msgs.RobotReport message) {
							if (!message.getStamp().isZero() && !initialLocations.containsKey(robotID)) {
								Quaternion quat = new Quaternion(message.getState().getPose().getOrientation().getX(), message.getState().getPose().getOrientation().getY(), message.getState().getPose().getOrientation().getZ(), message.getState().getPose().getOrientation().getW());
								Pose pose = new Pose(message.getState().getPose().getPosition().getX(), message.getState().getPose().getPosition().getY(), quat.getTheta());
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
							callComputeTaskService(message, robotID);
						}
					});
					
				}
				
			}

			@Override
			protected void loop() throws InterruptedException {
			    if (locationsFile != null && goalSequenceFile != null) {
			    	for (int robotID : robotIDs) {
			    		if (initialLocations.containsKey(robotID) && !computing.get(robotID) && tec.isFree(robotID)) {
			    			Mission m = Missions.getMission(robotID, missionNumber.get(robotID));
			    			missionNumber.put(robotID, (missionNumber.get(robotID)+1)%Missions.getMissions(robotID).size());
			    			if (m.getFromLocation() == null) {
			    				m.setFromLocation("currentLocation");
			    				m.setFromPose(tec.getRobotReport(robotID).getPose());
			    				callComputeTaskService(m);
			    				System.out.println("######################\n######################\n### SENDING MISSION:\n### " + m + "\n######################\n######################\n");
			    			}
			    		}
			    	}			    	
			    }
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
			CONTROL_PERIOD = params.getInteger("/" + node.getName() + "/control_period");
			TEMPORAL_RESOLUTION = params.getDouble("/" + node.getName() + "/temporal_resolution");
			MAX_ACCEL = params.getDouble("/" + node.getName() + "/forward_model_max_accel");
			MAX_VEL = params.getDouble("/" + node.getName() + "/forward_model_max_vel");
			locationsFile = params.getString("/" + node.getName() + "/locations_file");
			goalSequenceFile = params.getString("/" + node.getName() + "/goal_sequence_file");
		}
		catch (org.ros.exception.ParameterNotFoundException e) {
			System.out.println("== Parameter not found ==");
			e.printStackTrace();
		}
	}
	
	private void readGoalSequenceFile() {
		try {
			Scanner in = new Scanner(new FileReader(goalSequenceFile));
			while (in.hasNextLine()) {
				String line = in.nextLine().trim();
				if (line.length() != 0 && !line.startsWith("#")) {
					String[] oneline = line.split(" |\t");
					int robotID = Integer.parseInt(oneline[0]);
					String goalLocation = oneline[1];
					Mission m = new Mission(robotID, null, goalLocation, null, Missions.getLocation(goalLocation));
					Missions.putMission(m);
				}
			}
			in.close();
		}
		catch (FileNotFoundException e) { e.printStackTrace(); }

	}
	
	private void callComputeTaskService(geometry_msgs.PoseStamped goalPose, final int robotID) {
		
		computing.put(robotID, true);
		
		ServiceClient<ComputeTaskRequest, ComputeTaskResponse> serviceClient;
		try { serviceClient = node.newServiceClient("/robot" + robotID + "/compute_task", ComputeTask._TYPE); }
		catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
		
		if (!robotIDstoGoalIDs.keySet().contains(robotID)) robotIDstoGoalIDs.put(robotID, 1);
		else robotIDstoGoalIDs.put(robotID, robotIDstoGoalIDs.get(robotID)+1);
		final int goalID = robotIDstoGoalIDs.get(robotID);
		
		final ComputeTaskRequest request = serviceClient.newMessage();
		request.setStartFromCurrentState(true);
		RobotTarget rt = node.getTopicMessageFactory().newFromType(RobotTarget._TYPE);
		orunav_msgs.PoseSteering ps = node.getTopicMessageFactory().newFromType(orunav_msgs.PoseSteering._TYPE);
		geometry_msgs.Pose gpose = node.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
		gpose.setPosition(goalPose.getPose().getPosition());
		gpose.setOrientation(goalPose.getPose().getOrientation());
		ps.setPose(gpose);
		ps.setSteering(0.0);
		rt.setGoal(ps);
		rt.setRobotId(robotID);
		rt.setTaskId(goalID);
		rt.setGoalId(goalID);
		request.setTarget(rt);
		serviceClient.call(request, new ServiceResponseListener<ComputeTaskResponse>() {

			@Override
			public void onFailure(RemoteException arg0) {
				System.out.println("FAILED to call ComputeTask service for robot" + robotID + " (goalID: " + goalID + ")");				
			}

			@Override
			public void onSuccess(ComputeTaskResponse arg0) {
				System.out.println("Successfully called ComputeTask service for robot" + robotID + " (goalID: " + goalID + ")");
				
				tec.setCurrentTask(arg0.getTask().getTarget().getRobotId(), arg0.getTask());

				ArrayList<PoseSteering> path = new ArrayList<PoseSteering>();
				Pose fromPose = null;
				Pose toPose = null;
				for (int i = 0; i < arg0.getTask().getPath().getPath().size(); i++) {
					orunav_msgs.PoseSteering onePS = arg0.getTask().getPath().getPath().get(i);
					Quaternion quat = new Quaternion(onePS.getPose().getOrientation().getX(), onePS.getPose().getOrientation().getY(), onePS.getPose().getOrientation().getZ(), onePS.getPose().getOrientation().getW());
					PoseSteering ps = new PoseSteering(onePS.getPose().getPosition().getX(), onePS.getPose().getPosition().getY(), quat.getTheta(), onePS.getSteering());
					path.add(ps);
					if (i == 0) fromPose = ps.getPose();
					if (i == arg0.getTask().getPath().getPath().size()-1) toPose = ps.getPose();
				}
				PoseSteering[] pathArray = path.toArray(new PoseSteering[path.size()]);
				Mission m = new Mission(arg0.getTask().getTarget().getRobotId(), pathArray, "A", "B", fromPose, toPose);
				tec.addMissions(m);
				tec.computeCriticalSections();
				tec.startTrackingAddedMissions();
				computing.put(robotID, false);
			}
			
		});
	}
	
	private void callComputeTaskService(final Mission m) {
		
		final int robotID = m.getRobotID();
		computing.put(robotID, true);
		
		ServiceClient<ComputeTaskRequest, ComputeTaskResponse> serviceClient;
		try { serviceClient = node.newServiceClient("/robot" + robotID + "/compute_task", ComputeTask._TYPE); }
		catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
		
		if (!robotIDstoGoalIDs.keySet().contains(robotID)) robotIDstoGoalIDs.put(robotID, 1);
		else robotIDstoGoalIDs.put(robotID, robotIDstoGoalIDs.get(robotID)+1);
		final int goalID = robotIDstoGoalIDs.get(robotID);
		
		final ComputeTaskRequest request = serviceClient.newMessage();
		request.setStartFromCurrentState(true);
		RobotTarget rt = node.getTopicMessageFactory().newFromType(RobotTarget._TYPE);
		orunav_msgs.PoseSteering ps = node.getTopicMessageFactory().newFromType(orunav_msgs.PoseSteering._TYPE);
		geometry_msgs.Pose gpose = node.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
		gpose.getPosition().setX(m.getToPose().getX());
		gpose.getPosition().setY(m.getToPose().getY());
		Quaternion q = new Quaternion(m.getToPose().getTheta());
		gpose.getOrientation().setW(q.getW());
		gpose.getOrientation().setX(q.getX());
		gpose.getOrientation().setY(q.getY());
		gpose.getOrientation().setZ(q.getZ());
		ps.setPose(gpose);
		ps.setSteering(0.0);
		rt.setGoal(ps);
		rt.setRobotId(robotID);
		rt.setTaskId(goalID);
		rt.setGoalId(goalID);
		request.setTarget(rt);
		serviceClient.call(request, new ServiceResponseListener<ComputeTaskResponse>() {

			@Override
			public void onFailure(RemoteException arg0) {
				System.out.println("FAILED to call ComputeTask service for robot" + robotID + " (goalID: " + goalID + ")");				
			}

			@Override
			public void onSuccess(ComputeTaskResponse arg0) {
				System.out.println("Successfully called ComputeTask service for robot" + robotID + " (goalID: " + goalID + ")");
				
				tec.setCurrentTask(arg0.getTask().getTarget().getRobotId(), arg0.getTask());

				ArrayList<PoseSteering> path = new ArrayList<PoseSteering>();
				Pose fromPose = null;
				Pose toPose = null;
				for (int i = 0; i < arg0.getTask().getPath().getPath().size(); i++) {
					orunav_msgs.PoseSteering onePS = arg0.getTask().getPath().getPath().get(i);
					Quaternion quat = new Quaternion(onePS.getPose().getOrientation().getX(), onePS.getPose().getOrientation().getY(), onePS.getPose().getOrientation().getZ(), onePS.getPose().getOrientation().getW());
					PoseSteering ps = new PoseSteering(onePS.getPose().getPosition().getX(), onePS.getPose().getPosition().getY(), quat.getTheta(), onePS.getSteering());
					path.add(ps);
					if (i == 0) fromPose = ps.getPose();
					if (i == arg0.getTask().getPath().getPath().size()-1) toPose = ps.getPose();
				}
				PoseSteering[] pathArray = path.toArray(new PoseSteering[path.size()]);
				m.setPath(pathArray);
				tec.addMissions(m);
				tec.computeCriticalSections();
				tec.startTrackingAddedMissions();
				computing.put(robotID, false);
			}
			
		});
	}
}
