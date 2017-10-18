package se.oru.coordination.coordinator.ros_coordinator.simulation2D;

import java.util.Calendar;

import org.metacsp.multi.spatioTemporal.paths.Pose;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.util.AffineTransformation;

public abstract class WheelLoaderSimulator extends Thread {
	
	protected Pose[] poses = null;
	protected int currentPose = 0;
	protected int DELTA_CHANGE = 10000;
	protected long lastUpdate = -1;
	protected Geometry g = null;
	protected Geometry baseGeom = null;
	protected Coordinate[] footprint = null;
	
	public WheelLoaderSimulator(int period, Coordinate[] footprint, Pose ... poses) {
		this.poses = poses;
		this.DELTA_CHANGE = period;
		this.footprint = footprint;
		GeometryFactory gf = new GeometryFactory();
		baseGeom = gf.createPolygon(this.footprint);
	}
	
	public Pose getCurrentPoseOfWheelLoader() {
		return poses[currentPose];
	}
	
	public Geometry getCurrentGeometryOfWheelLoader() {
		return this.g;
	}
	
	public abstract void callback(); 
	
	public void run() {
		
		while(true) {
			long timeNow = Calendar.getInstance().getTimeInMillis();
			if (timeNow-lastUpdate > DELTA_CHANGE) {
				AffineTransformation at = new AffineTransformation();
				at.setToTranslation(poses[currentPose].getX(), poses[currentPose].getY());
				g = at.transform(baseGeom);
				currentPose = (currentPose+1)%poses.length;
				lastUpdate = timeNow;
			}
			callback();
			try { Thread.sleep(200); }
			catch (InterruptedException e) { e.printStackTrace(); }
		}
	}

}
