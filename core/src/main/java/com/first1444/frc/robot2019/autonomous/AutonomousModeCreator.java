package com.first1444.frc.robot2019.autonomous;

import com.first1444.sim.api.Transform2;
import me.retrodaredevil.action.Action;

public interface AutonomousModeCreator {

    /**
     *
     * @throws IllegalArgumentException Thrown if either startingPosition, gamePieceType, or level aren't supported by autonomousType
     * @throws NullPointerException if autonomousType, lineUpType, or startingOrientation is null
     * @return The autonomous action
     */
    Action createAction(AutonomousSettings autonomousSettings, Transform2 startingTransform);
}
