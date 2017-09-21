package se.oru.coordination.coordinator.ros_coordinator;
import java.io.Serializable;

public class IliadItem implements Serializable {
	
	public static enum ROTATION_TYPE {NONE, X, Y, Z, XZ, ZX}
	private static final long serialVersionUID = -828057091224081778L;
	private String name;
	private double x, y, z;
	private ROTATION_TYPE rotationType;
	
	private boolean isCylinder;
	
	public IliadItem(String name, double x, double y, double z, boolean isCylinder, ROTATION_TYPE rotType) {
		this.name = name;
		this.x = x;
		this.y = y;
		this.z = z;
		this.rotationType = rotType;
		this.isCylinder = isCylinder;
	}

	public String getName() {
		return name;
	}

	public ROTATION_TYPE getRotationType() {
		return rotationType;
	}

	public double getZ() {
		return z;
	}

	public double getY() {
		return y;
	}

	public boolean isCylinder() {
		return isCylinder;
	}

	public double getX() {
		return x;
	}

}
