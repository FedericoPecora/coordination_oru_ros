package se.oru.coordination.coordinator.util;

import java.io.File;

import org.metacsp.utility.logging.MetaCSPLogging;

public class BrowseLogs {

	public static void main(String[] args) {
		MetaCSPLogging.showLogs(System.getProperty("user.home") + File.separator + ".ros" + File.separator + "logs");
	}

}
