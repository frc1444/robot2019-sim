package com.first1444.frc.util.autonomous.creator;

import com.first1444.frc.robot2019.Robot;
import com.first1444.frc.robot2019.autonomous.actions.TurnToOrientation;
import com.first1444.frc.robot2019.autonomous.original.actions.GoStraight;
import com.first1444.sim.api.Rotation2;
import me.retrodaredevil.action.Action;

public class OriginalRobotSwerveDriveActionCreator implements OriginalSwerveDriveActionCreator {
    private final Robot robot;

    public OriginalRobotSwerveDriveActionCreator(Robot robot) {
        this.robot = robot;
    }
    @Override
    public Action createTurnToOrientation(Rotation2 desiredOrientation) {
        return new TurnToOrientation(desiredOrientation, robot.getDrive(), robot.getOrientation());
    }
    @Override
    public Action createGoStraight(double distanceMeters, double speed, Rotation2 angle) {
        return GoStraight.createGoStraightAtHeading(distanceMeters, speed, angle, null, robot.getDrive(), robot.getOrientation());
    }

    @Override
    public Action createGoStraight(double distanceMeters, double speed, Rotation2 angle, Rotation2 faceDirection) {
        return GoStraight.createGoStraightAtHeading(distanceMeters, speed, angle, faceDirection, robot.getDrive(), robot.getOrientation());
    }
}
