package com.first1444.frc.robot2019;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public interface ShuffleboardMap {
	ShuffleboardTab getUserTab();
	ShuffleboardTab getDevTab();
	ShuffleboardTab getDebugTab();
}
