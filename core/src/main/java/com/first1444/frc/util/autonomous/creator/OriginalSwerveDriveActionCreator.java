package com.first1444.frc.util.autonomous.creator;

import com.first1444.sim.api.Rotation2;
import me.retrodaredevil.action.Action;

@Deprecated
public interface OriginalSwerveDriveActionCreator {
    Action createTurnToOrientation(Rotation2 desiredOrientation);

    Action createGoStraight(double distanceMeters, double speed, Rotation2 angle);

    Action createGoStraight(double distanceMeters, double speed, Rotation2 angle, Rotation2 faceDirection);
}
