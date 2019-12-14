package com.first1444.frc.robot2019.autonomous;

import com.first1444.frc.robot2019.autonomous.options.AfterComplete;
import com.first1444.frc.robot2019.autonomous.options.AutonomousType;
import com.first1444.frc.robot2019.autonomous.options.LineUpType;
import com.first1444.frc.robot2019.autonomous.options.StartingPosition;
import com.first1444.frc.robot2019.deepspace.GamePieceType;
import com.first1444.frc.robot2019.deepspace.SlotLevel;
import com.first1444.sim.api.Rotation2;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import static java.util.Objects.requireNonNull;

public final class AutonomousSettings {
    @NotNull
    private final AutonomousType autonomousType;
    @Nullable
    private final StartingPosition startingPosition;
    @Nullable
    private final GamePieceType gamePieceType;
    @Nullable
    private final SlotLevel slotLevel;
    @NotNull
    private final LineUpType lineUpType;
    @Nullable
    private final AfterComplete afterComplete;
    @NotNull
    private final Rotation2 startingOrientation;

    /**
     * @param autonomousType The autonomous type
     * @param startingPosition The starting position of the robot. Can be null
     * @param gamePieceType The game piece type. Can be null
     * @param slotLevel The slot level to place the game piece type at. Can be null
     * @param startingOrientation The starting orientation of the robot
     * @throws NullPointerException if autonomousType, lineUpType, or startingOrientation is null
     */
    public AutonomousSettings(@NotNull AutonomousType autonomousType, @Nullable StartingPosition startingPosition, @Nullable GamePieceType gamePieceType, @Nullable SlotLevel slotLevel, @NotNull LineUpType lineUpType, @Nullable AfterComplete afterComplete, @NotNull Rotation2 startingOrientation) {
        this.autonomousType = requireNonNull(autonomousType);
        this.startingPosition = startingPosition;
        this.gamePieceType = gamePieceType;
        this.slotLevel = slotLevel;
        this.lineUpType = requireNonNull(lineUpType);
        this.afterComplete = afterComplete;
        this.startingOrientation = requireNonNull(startingOrientation);

        // TODO check for unsupported arguments // * @throws IllegalArgumentException Thrown if either startingPosition, gamePieceType, or level aren't supported by autonomousType
    }

    @NotNull
    public AutonomousType getAutonomousType() {
        return autonomousType;
    }

    @Nullable
    public StartingPosition getStartingPosition() {
        return startingPosition;
    }

    @Nullable
    public GamePieceType getGamePieceType() {
        return gamePieceType;
    }

    @Nullable
    public SlotLevel getSlotLevel() {
        return slotLevel;
    }

    @NotNull
    public LineUpType getLineUpType() {
        return lineUpType;
    }

    @Nullable
    public AfterComplete getAfterComplete() {
        return afterComplete;
    }

    @NotNull
    public Rotation2 getStartingOrientation() {
        return startingOrientation;
    }
}
