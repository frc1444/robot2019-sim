package com.first1444.frc.robot2019;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class DefaultShuffleboardMap implements ShuffleboardMap {
	private final ShuffleboardTab userTab;
	private final ShuffleboardTab devTab;
	private final ShuffleboardTab debugTab;
	public DefaultShuffleboardMap(){
		userTab = Shuffleboard.getTab("user");
		devTab = Shuffleboard.getTab("dev");
		debugTab = Shuffleboard.getTab("debug");
	}

	@Override
	public ShuffleboardTab getUserTab() {
		return userTab;
	}

	@Override
	public ShuffleboardTab getDevTab() {
		return devTab;
	}

	@Override
	public ShuffleboardTab getDebugTab() {
		return debugTab;
	}
}
