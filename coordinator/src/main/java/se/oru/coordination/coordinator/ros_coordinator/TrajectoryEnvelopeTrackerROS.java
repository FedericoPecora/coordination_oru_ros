package se.oru.coordination.coordinator.ros_coordinator;

import java.util.concurrent.TimeUnit;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.Quaternion;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.MessageListener;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Subscriber;

import orunav_msgs.ExecuteTask;
import orunav_msgs.ExecuteTaskRequest;
import orunav_msgs.ExecuteTaskResponse;
import orunav_msgs.Task;
import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.TrackingCallback;

public class TrajectoryEnvelopeTrackerROS extends AbstractTrajectoryEnvelopeTracker {

	protected static int goalID = 0;
	protected ConnectedNode node = null;
	protected RobotReport currentRR = null;
	protected Subscriber<orunav_msgs.RobotReport> subscriber = null;
	protected Task currentTask = null;

	public TrajectoryEnvelopeTrackerROS(TrajectoryEnvelope te, double temporalResolution, TrajectoryEnvelopeSolver solver, TrackingCallback cb, ConnectedNode connectedNode, Task currentTask) {
		super(te, temporalResolution, solver, 30, cb);
		this.node = connectedNode;
		this.currentTask = currentTask;
		if (currentTask == null) throw new Error("Trying to instantiate a TrajectoryEnvelopeTrackerROS for Robot" + te.getRobotID() + " with currentTask == " + currentTask);
		subscriber = connectedNode.newSubscriber("robot"+te.getRobotID()+"/report", orunav_msgs.RobotReport._TYPE);
	    subscriber.addMessageListener(new MessageListener<orunav_msgs.RobotReport>() {
	      @Override
	      public void onNewMessage(orunav_msgs.RobotReport message) {
	    	  Quaternion quat = new Quaternion(message.getState().getPose().getOrientation().getX(), message.getState().getPose().getOrientation().getY(), message.getState().getPose().getOrientation().getZ(), message.getState().getPose().getOrientation().getW());
	    	  Pose pose = new Pose(message.getState().getPose().getPosition().getX(), message.getState().getPose().getPosition().getY(), quat.getTheta());
	    	  int index = message.getSequenceNum();
	    	  currentRR = new RobotReport(pose, index, -1.0, -1.0, -1);
	    	  onPositionUpdate();
	      }
	    });
	}
	
	@Override
	public long getCurrentTimeInMillis() {
		return TimeUnit.NANOSECONDS.toMillis(node.getCurrentTime().totalNsecs());
	}

	@Override
	public RobotReport getRobotReport() {
		while (currentRR == null) {
			try { Thread.sleep(100); }
			catch (InterruptedException e) { e.printStackTrace(); }
		}
		return currentRR;
	}

	@Override
	public void onPositionUpdate() { }

	@Override
	public void setCriticalPoint(int arg0) {
		ServiceClient<ExecuteTaskRequest, ExecuteTaskResponse> serviceClient;
		try { serviceClient = node.newServiceClient("/robot" + currentTask.getTarget().getRobotId() + "/execute_task", ExecuteTask._TYPE); }
		catch (NullPointerException npe) { 
			System.out.println("WARNING: trying to set critical point (" + arg0 + ") when currentTask is " + currentTask);
			throw new RosRuntimeException(npe);
		}
		catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
		final ExecuteTaskRequest request = serviceClient.newMessage();
		currentTask.setUpdate(true);
		currentTask.setCriticalPoint(arg0);
		request.setTask(currentTask);
		serviceClient.call(request, new ServiceResponseListener<ExecuteTaskResponse>() {
			@Override
			public void onSuccess(ExecuteTaskResponse response) {
				System.out.println("Updated execution of goal " + currentTask.getTarget().getGoalId() + " for robot " + currentTask.getTarget().getRobotId());
			}
			@Override
			public void onFailure(RemoteException arg0) {
				System.out.println("Failed to update execution of goal " + currentTask.getTarget().getGoalId() + " for robot " + currentTask.getTarget().getRobotId());
			}
		});
	}

	@Override
	public void startTracking() { }
	
	@Override
	protected void finishTracking() {
		super.finishTracking();
		subscriber.shutdown();
	}

}
