package se.oru.coordination.coordinator.ros_coordinator;

import java.util.concurrent.TimeUnit;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.Quaternion;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.ros.message.MessageListener;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.TrackingCallback;

public class TrajectoryEnvelopeTrackerROS extends AbstractTrajectoryEnvelopeTracker {

	protected static int goalID = 0;
	protected ConnectedNode node = null;
	protected RobotReport currentRR = null;
	protected Subscriber<orunav_msgs.RobotReport> subscriber = null;

	public TrajectoryEnvelopeTrackerROS(TrajectoryEnvelope te, double temporalResolution, TrajectoryEnvelopeSolver solver, final int trackingPeriodInMillis, TrackingCallback cb, ConnectedNode connectedNode) {
		super(te, temporalResolution, solver, trackingPeriodInMillis, cb);
		this.node = connectedNode;
		subscriber = connectedNode.newSubscriber("robot"+te.getRobotID()+"/report", orunav_msgs.RobotReport._TYPE);
	    subscriber.addMessageListener(new MessageListener<orunav_msgs.RobotReport>() {
	      @Override
	      public void onNewMessage(orunav_msgs.RobotReport message) {
	    	  Quaternion quat = new Quaternion(message.getState().getPose().getOrientation().getX(), message.getState().getPose().getOrientation().getY(), message.getState().getPose().getOrientation().getZ(), message.getState().getPose().getOrientation().getW());
	    	  Pose pose = new Pose(message.getState().getPose().getPosition().getX(), message.getState().getPose().getPosition().getY(), quat.getTheta());
	    	  int index = message.getSequenceNum();
	    	  currentRR = new RobotReport(pose, index, -1.0, -1.0, trackingPeriodInMillis);
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
	public void onPositionUpdate() {
		// TODO Auto-generated method stub
	}

	@Override
	public void setCriticalPoint(int arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void startTracking() {
		// TODO Auto-generated method stub

	}
	
	@Override
	protected void finishTracking() {
		super.finishTracking();
		subscriber.shutdown();
	}

}
