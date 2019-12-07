package com.first1444.frc.robot2019;


import com.first1444.dashboard.shuffleboard.ShuffleboardContainer;

public interface ShuffleboardMap {
    ShuffleboardContainer getUserTab();
    ShuffleboardContainer getDevTab();
    ShuffleboardContainer getDebugTab();
}
