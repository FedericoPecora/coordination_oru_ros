package se.oru.coordination.coordinator.util;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.logging.Logger;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.utility.logging.MetaCSPLogging;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordinator.ros_coordinator.IliadItem;
import se.oru.coordination.coordinator.ros_coordinator.IliadItem.ROTATION_TYPE;
import se.oru.coordination.coordinator.ros_coordinator.IliadMission;
import se.oru.coordination.coordinator.ros_coordinator.IliadMission.OPERATION_TYPE;

public class IliadMissions extends Missions {
	
	private static Logger metaCSPLogger = MetaCSPLogging.getLogger(IliadMission.class);
	
	/**
	 * Load mission data from XML file.
	 * @param fileName The file to load the data from.
	 */
	public static void loadIliadMissions(String fileName) {
		try {
			
			ArrayList<IliadMission> missions = new ArrayList<IliadMission>();
			
			File fXmlFile = new File(fileName);
			DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
			DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
			Document doc = dBuilder.parse(fXmlFile);
			doc.getDocumentElement().normalize();

			NodeList nList = doc.getElementsByTagName("Mission");

			for (int temp = 0; temp < nList.getLength(); temp++) {
				Node oneMissionNode = nList.item(temp);
				if (oneMissionNode.getNodeType() == Node.ELEMENT_NODE) {
					Element eElement = (Element)oneMissionNode;
					int robotID = Integer.parseInt(eElement.getElementsByTagName("robotID").item(0).getTextContent());
					boolean repeatMission = true;
					if (eElement.getElementsByTagName("repeat").getLength() > 0) repeatMission = Boolean.parseBoolean(eElement.getElementsByTagName("repeat").item(0).getTextContent());
					String fromLocation = eElement.getElementsByTagName("fromLocation").item(0).getTextContent();
					String toLocation = eElement.getElementsByTagName("toLocation").item(0).getTextContent();
					Pose fromPose = null;
					Pose goalPose = null;
					OPERATION_TYPE missionType = OPERATION_TYPE.valueOf(eElement.getAttributes().getNamedItem("type").getTextContent());
					NodeList poseList = eElement.getElementsByTagName("Pose");
					if (poseList.getLength() != 0) {
						for (int j = 0; j < poseList.getLength(); j++) {
							Node onePose = poseList.item(j);
							Element onePoseElement = (Element)onePose;
							if (onePoseElement.getAttributes().getNamedItem("name").getTextContent().equals("fromPose")) {
								double x = Double.parseDouble(((Element)onePoseElement.getElementsByTagName("x").item(0)).getTextContent());
								double y = Double.parseDouble(((Element)onePoseElement.getElementsByTagName("y").item(0)).getTextContent());
								double theta = Double.parseDouble(((Element)onePoseElement.getElementsByTagName("theta").item(0)).getTextContent());
								fromPose = new Pose(x,y,theta);
							}
							else if (onePoseElement.getAttributes().getNamedItem("name").getTextContent().equals("toPose")) {
								double x = Double.parseDouble(((Element)onePoseElement.getElementsByTagName("x").item(0)).getTextContent());
								double y = Double.parseDouble(((Element)onePoseElement.getElementsByTagName("y").item(0)).getTextContent());
								double theta = Double.parseDouble(((Element)onePoseElement.getElementsByTagName("theta").item(0)).getTextContent());
								goalPose = new Pose(x,y,theta);
							}
						}
					}
					else {
						if (Missions.locations.containsKey(fromLocation)) fromPose = Missions.getLocation(fromLocation);
						else fromPose = null;
						if (Missions.locations.containsKey(toLocation)) goalPose = Missions.getLocation(toLocation);
						else goalPose = null;
					}
					
					ArrayList<IliadItem> items = new ArrayList<IliadItem>();
					NodeList itemList = eElement.getElementsByTagName("IliadItem");
					for (int j = 0; j < itemList.getLength(); j++) {
						Node oneItem = itemList.item(j);
						Element oneItemElement = (Element)oneItem;
						String itemName = oneItemElement.getAttributes().getNamedItem("name").getTextContent();
						double x = Double.parseDouble(((Element)oneItemElement.getElementsByTagName("x").item(0)).getTextContent());
						double y = Double.parseDouble(((Element)oneItemElement.getElementsByTagName("y").item(0)).getTextContent());
						double z = Double.parseDouble(((Element)oneItemElement.getElementsByTagName("z").item(0)).getTextContent());
						ROTATION_TYPE rotType = ROTATION_TYPE.valueOf(((Element)oneItemElement.getElementsByTagName("rotationType").item(0)).getTextContent());
						IliadItem item = new IliadItem(itemName, x, y, z, rotType);
						items.add(item);
					}
							
					if (missionType.equals(OPERATION_TYPE.PICK_ITEMS)) {
						IliadItem[] itemsArray = items.toArray(new IliadItem[items.size()]);
						missions.add(new IliadMission(robotID, fromLocation, toLocation, fromPose, goalPose, repeatMission, itemsArray));
					}
					else {
						missions.add(new IliadMission(robotID, fromLocation, toLocation, fromPose, goalPose, missionType, repeatMission));
					}
				}				
			}
			for (IliadMission mission : missions) {
				Missions.putMission(mission);
				metaCSPLogger.info("Loaded mission " + mission);
			}
		}
		catch (IOException e) { e.printStackTrace(); } catch (ParserConfigurationException e) { e.printStackTrace(); }
		catch (SAXException e) { e.printStackTrace(); }
	}
	
	public static void main(String[] args) {
		loadIliadMissions(".."+File.separator+"missions"+File.separator+"NCFMScenarioMS1Missions.xml");
	}

}
