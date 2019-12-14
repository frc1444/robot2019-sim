package com.first1444.frc.robot2019.autonomous.creator;

import com.first1444.frc.robot2019.Robot;
import com.first1444.frc.robot2019.autonomous.creator.action.OperatorActionCreator;
import com.first1444.frc.robot2019.autonomous.creator.action.RobotOperatorActionCreator;
import com.first1444.frc.robot2019.autonomous.creator.action.RobotVisionPlacementCreator;
import com.first1444.frc.robot2019.autonomous.creator.action.VisionPlacementCreator;
import com.first1444.frc.util.autonomous.creator.DefaultSwerveDriveActionCreator;
import com.first1444.frc.util.autonomous.creator.FrcLogActionCreator;
import com.first1444.frc.util.autonomous.creator.LogActionCreator;
import com.first1444.frc.util.autonomous.creator.SwerveDriveActionCreator;

public class RobotAutonomousActionCreator implements AutonomousActionCreator {

    private final LogActionCreator logActionCreator;
    private final SwerveDriveActionCreator driveCreator;
    private final OperatorActionCreator operatorActionCreator;
    private final VisionPlacementCreator visionPlacementCreator;

    public RobotAutonomousActionCreator(Robot robot) {
        logActionCreator = new FrcLogActionCreator(System.out, robot.getLogger());
        driveCreator = new DefaultSwerveDriveActionCreator(robot.getDrive(), robot.getOrientation(), robot.getAbsoluteDistanceAccumulator());
        operatorActionCreator = new RobotOperatorActionCreator(robot);
        visionPlacementCreator = new RobotVisionPlacementCreator(robot);
    }

    @Override
    public LogActionCreator getLogCreator() {
        return logActionCreator;
    }

    @Override
    public SwerveDriveActionCreator getDriveCreator() {
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
