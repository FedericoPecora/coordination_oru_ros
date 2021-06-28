package se.oru.coordination.coordinator.ros_coordinator;

import java.util.ArrayList;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import se.oru.coordination.coordination_oru.Mission;

public class IliadMission extends Mission {

	public static enum OPERATION_TYPE {_IGNORE_, NO_OPERATION, UNLOAD_PALLET, LOAD_PALLET, LOAD_DETECT, ACTIVATE_SUPPORT_LEGS, LOAD_DETECT_ACTIVE, PICK_ITEMS, UNWRAP_PALLET};
	public static enum LOAD_TYPE {EMPTY, EUR_PALLET, HALF_PALLET, UNKNOWN};
	private IliadItem[] items;
	private OPERATION_TYPE startOp;
	private OPERATION_TYPE goalOp;
	private LOAD_TYPE goalLoad;
	private boolean repeat = true;
	
	public IliadMission(int robotID, PoseSteering[] path, String fromLocation, String toLocation, Pose fromPose, Pose toPose, OPERATION_TYPE startOp, boolean repeat, IliadItem ... items) {
		super(robotID, path, fromLocation, toLocation, fromPose, toPose);
		this.items = items;
		this.startOp = startOp;
		this.goalOp = OPERATION_TYPE.PICK_ITEMS;
		System.out.println("Default goal load: assuming EUR_PALLET.");
		this.goalLoad = LOAD_TYPE.EUR_PALLET;
		this.repeat = repeat;
	}

	public IliadMission(int robotID,  PoseSteering[] path, String fromLocation, String toLocation, Pose fromPose, Pose toPose, OPERATION_TYPE startOp, OPERATION_TYPE goalOp, LOAD_TYPE goalLoad, boolean repeat) {
		super(robotID, path, fromLocation, toLocation, fromPose, toPose);
		this.repeat = repeat;
		this.startOp = startOp;
		this.goalOp = goalOp;
		this.goalLoad = goalLoad;
	}
	
	public IliadMission(int robotID, PoseSteering[] path, String fromLocation, String toLocation, Pose fromPose, Pose toPose, boolean repeat) {
		super(robotID, path, fromLocation, toLocation, fromPose, toPose);
		this.startOp = OPERATION_TYPE.NO_OPERATION;
		this.goalOp = OPERATION_TYPE.NO_OPERATION;
		this.goalLoad = LOAD_TYPE.UNKNOWN;
		this.repeat = repeat;
	}
	
	public boolean repeatMission() {
		return this.repeat;
	}
	
	public void setPath(PoseSteering[] path) {
		this.path = path;
	}
	
	public void setStartPose(Pose startPose) {
		this.fromPose = startPose;
	}
	
	public OPERATION_TYPE getStartOperation() {
		return this.startOp;
	}
	
	public OPERATION_TYPE getGoalOperation() {
		return this.goalOp;
	}
	
	public LOAD_TYPE getGoalLoad() {
		return this.goalLoad;
	}
	
	public IliadItem[] getItems() {
		return this.items;
	}
	
	public IliadItem getItem(int i) {
		if (i >= 0 && i < items.length) return items[i];
		throw new Error("Item " + i + " does not exist");
	}

	public IliadItem[] getItems(String name) {
		ArrayList<IliadItem> ret = new ArrayList<IliadItem>();
		for (IliadItem item : items) if (item.getName().equals(name)) ret.add(item);
		if (ret.isEmpty()) return null;
		return ret.toArray(new IliadItem[ret.size()]);
	}
	
	public String toString() {
		//return this.operationType + " " + (this.items != null ? Arrays.toString(items) + " " : "") + super.toString();
		return "startOp=" + this.getStartOperation() + ", " + "goalOp=" + this.getGoalOperation() + ", " + (this.items != null ? items.length + " items " : "") + super.toString();
	}
	
	public String toXML() {
		String ret = "<Mission startOp=\"" + this.getStartOperation() + "\">\n";
		ret += "goalOp=\"" + this.getGoalOperation() + "\">\n";
		ret += "   <robotID>" + this.robotID + "</robotID>\n";
		ret += "   <fromLocation>" + this.getFromLocation() + "</fromLocation>\n";
		ret += "   <toLocation>" + this.getToLocation() + "</toLocation>\n";
		ret += "   <Pose name=\"startPose\">\n";
		ret += "      <x>" + this.getFromPose().getX() + "</x>\n";
		ret += "      <y>" + this.getFromPose().getY() + "</y>\n";
		ret += "      <theta>" + this.getFromPose().getTheta() + "</theta>\n";
		ret += "   </Pose>\n";
		ret += "   <Pose name=\"toPose\">\n";
		ret += "      <x>" + this.getToPose().getX() + "</x>\n";
		ret += "      <y>" + this.getToPose().getY() + "</y>\n";
		ret += "      <theta>" + this.getToPose().getTheta() + "</theta>\n";
		ret += "   </Pose>\n";
		if (this.getItems() != null && this.getItems().length > 0) {
			ret += "   <IliadItems>\n";
			for (IliadItem item : this.getItems()) {
				ret += "      <IliadItem name=\"" + item.getName() + "\">\n";
				ret += "         <x>" + item.getX() + "</x>\n";
				ret += "         <y>" + item.getY() + "</y>\n";
				ret += "         <z>" + item.getY() + "</z>\n";
				ret += "         <rotationType>" + item.getRotationType() + "</rotationType>\n";
				ret += "      </IliadItem>\n";
			}
			ret += "   </IliadItems>\n";
		}
		ret += "</Mission>\n";
		
		return ret;
	}

}
