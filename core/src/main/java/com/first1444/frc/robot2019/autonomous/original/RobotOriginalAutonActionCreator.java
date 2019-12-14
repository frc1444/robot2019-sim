package com.first1444.frc.robot2019.autonomous.original;

import com.first1444.frc.robot2019.Robot;
import com.first1444.frc.robot2019.autonomous.creator.action.OperatorActionCreator;
import com.first1444.frc.robot2019.autonomous.creator.action.RobotOperatorActionCreator;
import com.first1444.frc.robot2019.autonomous.creator.action.RobotVisionPlacementCreator;
import com.first1444.frc.robot2019.autonomous.creator.action.VisionPlacementCreator;
import com.first1444.frc.util.autonomous.creator.*;

@Deprecated
public class RobotOriginalAutonActionCreator implements OriginalAutonActionCreator {
    private final LogActionCreator logActionCreator;
    private final OriginalSwerveDriveActionCreator driveCreator;
    private final OperatorActionCreator operatorActionCreator;
    private final VisionPlacementCreator visionPlacementCreator;

    public RobotOriginalAutonActionCreator(Robot robot) {
        logActionCreator = new FrcLogActionCreator(System.out, robot.getLogger());
        driveCreator = new OriginalRobotSwerveDriveActionCreator(robot);
        operatorActionCreator = new RobotOperatorActionCreator(robot);
        visionPlacementCreator = new RobotVisionPlacementCreator(robot);
    }

    @Override
    public LogActionCreator getLogCreator() {
        return logActionCreator;
    }

    @Override
    public OriginalSwerveDriveActionCreator getDriveCreator() {
        return driveCreator;
    }

    @Override
    public OperatorActionCreator getOperatorCreator() {
        return operatorActionCreator;
    }

    @Override
    public VisionPlacementCreator getVisionCreator() {
        return visionPlacementCreator;
    }
}
