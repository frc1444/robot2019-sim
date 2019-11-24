package com.first1444.frc.robot2019;

import com.first1444.dashboard.shuffleboard.Shuffleboard;
import com.first1444.dashboard.shuffleboard.ShuffleboardContainer;

public class DefaultShuffleboardMap implements ShuffleboardMap {
	private final ShuffleboardContainer userTab;
	private final ShuffleboardContainer devTab;
	private final ShuffleboardContainer debugTab;
	public DefaultShuffleboardMap(Shuffleboard shuffleboard){
		userTab = shuffleboard.get("user");
		devTab = shuffleboard.get("dev");
		debugTab = shuffleboard.get("debug");
	}

	@Override
	public ShuffleboardContainer getUserTab() {
		return userTab;
	}

	@Override
	public ShuffleboardContainer getDevTab() {
		return devTab;
	}

	@Override
	public ShuffleboardContainer getDebugTab() {
		return debugTab;
	}
}
