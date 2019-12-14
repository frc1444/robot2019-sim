package com.first1444.frc.robot2019.autonomous.creator;

import com.first1444.frc.robot2019.autonomous.creator.action.OperatorActionCreator;
import com.first1444.frc.robot2019.autonomous.creator.action.VisionPlacementCreator;
import com.first1444.frc.util.autonomous.creator.LogActionCreator;
import com.first1444.frc.util.autonomous.creator.SwerveDriveActionCreator;

public interface AutonomousActionCreator {
    LogActionCreator getLogCreator();
    SwerveDriveActionCreator getDriveCreator();
    OperatorActionCreator getOperatorCreator();
    VisionPlacementCreator getVisionCreator();
}
