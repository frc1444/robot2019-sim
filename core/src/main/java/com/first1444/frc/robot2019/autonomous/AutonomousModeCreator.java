package com.first1444.frc.robot2019.autonomous;

import com.first1444.frc.robot2019.autonomous.options.AfterComplete;
import com.first1444.frc.robot2019.autonomous.options.AutonomousType;
import com.first1444.frc.robot2019.autonomous.options.LineUpType;
import com.first1444.frc.robot2019.autonomous.options.StartingPosition;
import com.first1444.frc.robot2019.deepspace.GamePieceType;
import com.first1444.frc.robot2019.deepspace.SlotLevel;
import com.first1444.sim.api.Rotation2;
import me.retrodaredevil.action.Action;

public interface AutonomousModeCreator {

    /**
     *
     * @param autonomousType The autonomous type
     * @param startingPosition The starting position of the robot. Can be null
     * @param gamePieceType The game piece type. Can be null
     * @param slotLevel The slot level to place the game piece type at. Can be null
     * @param startingOrientation The starting orientation of the robot
     * @throws IllegalArgumentException Thrown if either startingPosition, gamePieceType, or level aren't supported by autonomousType
     * @throws NullPointerException if autonomousType, lineUpType, or startingOrientation is null
     * @return The autonomous action
     */
    Action createAction(
            AutonomousType autonomousType,
            StartingPosition startingPosition,
            GamePieceType gamePieceType,
            SlotLevel slotLevel,
            LineUpType lineUpType,
            AfterComplete afterComplete,
            Rotation2 startingOrientation);
}
