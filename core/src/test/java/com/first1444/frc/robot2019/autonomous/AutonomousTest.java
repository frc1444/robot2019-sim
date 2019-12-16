package com.first1444.frc.robot2019.autonomous;

import com.first1444.frc.robot2019.Constants;
import com.first1444.frc.robot2019.autonomous.options.AfterComplete;
import com.first1444.frc.robot2019.autonomous.options.AutonomousType;
import com.first1444.frc.robot2019.autonomous.options.LineUpType;
import com.first1444.frc.robot2019.autonomous.options.StartingPosition;
import com.first1444.frc.robot2019.autonomous.original.OriginalAutonomousModeCreator;
import com.first1444.frc.robot2019.autonomous.original.TestOriginalAutonActionCreator;
import com.first1444.frc.robot2019.deepspace.GamePieceType;
import com.first1444.frc.robot2019.deepspace.SlotLevel;
import com.first1444.sim.api.Rotation2;
import me.retrodaredevil.action.Action;
import org.junit.jupiter.api.Test;

import java.util.stream.Collectors;
import java.util.stream.Stream;

public class AutonomousTest {
	public static void main(String[] args){
		final AutonomousModeCreator modeCreator = new OriginalAutonomousModeCreator(new TestOriginalAutonActionCreator(System.out), Constants.Dimensions.INSTANCE);
		runMode(modeCreator, AutonomousType.OFF_CENTER_CARGO_SHIP, StartingPosition.MIDDLE_RIGHT, GamePieceType.HATCH, SlotLevel.LEVEL1, LineUpType.NO_VISION, null, Rotation2.DEG_90);
		runMode(modeCreator, AutonomousType.DO_NOTHING, null, null, null, LineUpType.NO_VISION, null, Rotation2.DEG_90);
		runMode(modeCreator, AutonomousType.CROSS_LINE_FORWARD, null, null, null, LineUpType.NO_VISION, null, Rotation2.DEG_90);
		runMode(modeCreator, AutonomousType.CROSS_LINE_SIDE, StartingPosition.RIGHT, null, null, LineUpType.NO_VISION, null, Rotation2.DEG_90);
		for(LineUpType lineUpType : LineUpType.values()) {
			
			runMode(modeCreator, AutonomousType.SIDE_CARGO_SHIP, StartingPosition.RIGHT, GamePieceType.HATCH, SlotLevel.LEVEL1, lineUpType, null, Rotation2.DEG_180);
			runMode(modeCreator, AutonomousType.SIDE_CARGO_SHIP, StartingPosition.LEFT, GamePieceType.HATCH, SlotLevel.LEVEL1, lineUpType, null, Rotation2.ZERO);
			
			runMode(modeCreator, AutonomousType.SIDE_CARGO_SHIP, StartingPosition.RIGHT, GamePieceType.CARGO, SlotLevel.LEVEL1, lineUpType, null, Rotation2.ZERO);
			runMode(modeCreator, AutonomousType.SIDE_CARGO_SHIP, StartingPosition.LEFT, GamePieceType.CARGO, SlotLevel.LEVEL1, lineUpType, null, Rotation2.DEG_180);
			
			runMode(modeCreator, AutonomousType.SIDE_ROCKET, StartingPosition.RIGHT, GamePieceType.HATCH, SlotLevel.LEVEL1, lineUpType, null, Rotation2.DEG_90);
		}
	}
	private static void runMode(AutonomousModeCreator modeCreator, AutonomousType autonomousType,
								StartingPosition startingPosition, GamePieceType gamePieceType,
								SlotLevel slotLevel, LineUpType lineUpType, AfterComplete afterComplete, Rotation2 startingOrientation){
		System.out.println(autonomousType.getName());
		System.out.println(startingPosition);
		System.out.println(gamePieceType);
		System.out.println(slotLevel);
		System.out.println(lineUpType);
		System.out.println(afterComplete);
		System.out.println(startingOrientation);
		runUntilDone(modeCreator.createAction(new AutonomousSettings(autonomousType, startingPosition, gamePieceType, slotLevel, lineUpType, afterComplete, startingOrientation), ));
		System.out.println();
	}
	private static void runUntilDone(Action action){
		System.out.println("Starting");
		do {
			action.update();
		} while (!action.isDone());
		action.end();
		System.out.println("Ended!");
	}
	@Test
	void testAllAuto(){
		final AutonomousModeCreator modeCreator = new OriginalAutonomousModeCreator(new TestOriginalAutonActionCreator(System.out), Constants.Dimensions.INSTANCE);
		for(AutonomousType type : AutonomousType.values()) for(GamePieceType gamePiece : type.getGamePieces()) for(StartingPosition startingPosition : type.getStartingPositions()) for(SlotLevel slotLevel : type.getSlotLevels()) for(LineUpType lineUpType : type.getLineUpTypes())
			for(AfterComplete afterComplete : Stream.concat(type.getAfterCompleteOptions().stream(), Stream.of((AfterComplete) null)).collect(Collectors.toList()))
		{
			for(Rotation2 startingOrientation : new Rotation2[] {Rotation2.ZERO, Rotation2.DEG_90, Rotation2.DEG_180, Rotation2.DEG_270}) {
				runMode(modeCreator, type, startingPosition, gamePiece, slotLevel, lineUpType, afterComplete, startingOrientation);
			}
		}
	}
}
