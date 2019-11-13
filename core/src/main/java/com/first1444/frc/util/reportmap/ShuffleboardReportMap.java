package com.first1444.frc.util.reportmap;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

import java.util.HashMap;
import java.util.Map;

public class ShuffleboardReportMap implements ReportMap {
	
	private final ShuffleboardContainer container;
	private final Map<String, SimpleWidget> widgetMap = new HashMap<>();
	
	public ShuffleboardReportMap(ShuffleboardContainer container) {
		this.container = container;
	}
	
	@Override
	public void report(String key, String value) {
		final SimpleWidget widget = widgetMap.computeIfAbsent(key, _key -> container.add(key, value));
		final NetworkTableEntry entry = widget.getEntry();
		if(!entry.getString(null).equals(value)){
			entry.setString(value);
		}
	}
}
