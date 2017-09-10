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

package se.oru.coordination.coordinator.ros_coordinator;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
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
				
				//Setup a simple GUI (null means empty map, otherwise provide yaml file)
				tec.setupGUI(null);
				
				//Set the footprint of the robots
				tec.setDefaultFootprint(footprintCoords);
				
				for (final int robotID : robotIDs) {
					
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
		}
		catch (org.ros.exception.ParameterNotFoundException e) {
			System.out.println("== Parameter not found ==");
			e.printStackTrace();
		}
	}
	
	private void callComputeTaskService(geometry_msgs.PoseStamped goalPose, final int robotID) {
		
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
				callExecuteTaskService(arg0.getTask());
			}
			
		});
	}
	
	private void callExecuteTaskService(final Task task) {

		ServiceClient<ExecuteTaskRequest, ExecuteTaskResponse> serviceClient;
		try { serviceClient = node.newServiceClient("/robot" + task.getTarget().getRobotId() + "/execute_task", ExecuteTask._TYPE); }
		catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
		final ExecuteTaskRequest request = serviceClient.newMessage();
		task.setUpdate(false);
		CoordinatorTimeVec cts = computeCTsFromDTs(task.getDts());
		task.setCts(cts);
		
		//Operations used by the current execution service
		Operation startOp = node.getTopicMessageFactory().newFromType(Operation._TYPE);
		startOp.setOperation(Operation.NO_OPERATION);
		task.getTarget().setStartOp(startOp);
		Operation goalOp = node.getTopicMessageFactory().newFromType(Operation._TYPE);
		goalOp.setOperation(Operation.NO_OPERATION);
		task.getTarget().setGoalOp(goalOp);
		
		request.setTask(task);
		//tec.setCurrentTask(task.getTarget().getRobotId(), task);

		serviceClient.call(request, new ServiceResponseListener<ExecuteTaskResponse>() {
			@Override
			public void onSuccess(ExecuteTaskResponse response) {
					System.out.println("Started execution of goal " + task.getTarget().getGoalId() + " for robot " + task.getTarget().getRobotId());
					tec.startTrackingAddedMissions();
			}
			@Override
			public void onFailure(RemoteException arg0) {
				System.out.println("Failed to start execution of goal " + task.getTarget().getGoalId() + " for robot " + task.getTarget().getRobotId());
			}
		});		
		
	}
	
	private CoordinatorTimeVec computeCTsFromDTs(DeltaTVec dts) {
		CoordinatorTimeVec cts = node.getTopicMessageFactory().newFromType(CoordinatorTimeVec._TYPE);
		cts.setGoalId(dts.getGoalId());
		cts.setId(dts.getTrajId());
		ArrayList<CoordinatorTime> ctList = new ArrayList<CoordinatorTime>();
//		double currentTime = node.getCurrentTime().toSeconds();
//		double[] fastDTs = dts.getDts().get(0).getDt();
//		double[] slowDTs = dts.getDts().get(1).getDt();
//		double[] fastCTs = new double[fastDTs.length];
//		double[] slowCTs = new double[slowDTs.length];
		CoordinatorTime ctFast = node.getTopicMessageFactory().newFromType(CoordinatorTime._TYPE);
		CoordinatorTime ctSlow = node.getTopicMessageFactory().newFromType(CoordinatorTime._TYPE);
//		fastCTs[0] = currentTime;
//		slowCTs[0] = currentTime;
//		for (int i = 1; i < fastDTs.length; i++) {
//			fastCTs[i] = fastDTs[i]+fastCTs[i-1];
//		    slowCTs[i] = slowDTs[i]+slowCTs[i-1];
//		}
//		ctFast.setT(fastCTs);
//		ctSlow.setT(slowCTs);
		ctFast.setT(new double[] {-1, -1});
		ctSlow.setT(new double[] {-1, -1});
		ctList.add(ctFast);
		ctList.add(ctSlow);
		cts.setTs(ctList);
		return cts;
	}


}
