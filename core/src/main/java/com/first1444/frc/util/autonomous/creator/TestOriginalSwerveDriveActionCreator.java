package com.first1444.frc.util.autonomous.creator;

import com.first1444.frc.robot2019.Constants;
import com.first1444.sim.api.Rotation2;
import me.retrodaredevil.action.Action;

public class TestOriginalSwerveDriveActionCreator implements OriginalSwerveDriveActionCreator {
    private final LogActionCreator logActionCreator;

    public TestOriginalSwerveDriveActionCreator(LogActionCreator logActionCreator) {
        this.logActionCreator = logActionCreator;
    }


    @Override
    public Action createTurnToOrientation(Rotation2 desiredOrientation) {
        return logActionCreator.createLogMessageAction("Turning to orientation: " + desiredOrientation);
    }

    @Override
    public Action createGoStraight(double distanceMeters, double speed, Rotation2 angle) {
        return logActionCreator.createLogMessageAction("Going straight for " + distanceMeters + " meters at " + Constants.DECIMAL_FORMAT.format(speed)
            + " with " + angle + " heading.");
    }

    @Override
    public Action createGoStraight(double distanceMeters, double speed, Rotation2 angle, Rotation2 faceDirection) {
        return logActionCreator.createLogMessageAction("Going straight for " + distanceMeters + " meters at " + Constants.DECIMAL_FORMAT.format(speed)
            + " with " + angle + " heading while facing " + faceDirection);
    }

}
