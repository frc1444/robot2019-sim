package com.first1444.frc.robot2019.autonomous;

import me.retrodaredevil.action.Action;
import org.jetbrains.annotations.NotNull;

public interface AutonomousModeCreator {

    /**
     *
     * @throws IllegalArgumentException Thrown if either startingPosition, gamePieceType, or level aren't supported by autonomousType
     * @throws NullPointerException if autonomousType, lineUpType, or startingOrientation is null
     * @return The autonomous action
     */
    @NotNull
    Action createAction(@NotNull AutonomousSettings autonomousSettings);
}
