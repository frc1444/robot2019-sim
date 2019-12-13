package com.first1444.frc.robot2019.autonomous.original;

import com.first1444.frc.robot2019.RobotDimensions;
import com.first1444.frc.robot2019.autonomous.AutonomousModeCreator;
import com.first1444.frc.robot2019.autonomous.options.AfterComplete;
import com.first1444.frc.robot2019.autonomous.options.AutonomousType;
import com.first1444.frc.robot2019.autonomous.options.LineUpType;
import com.first1444.frc.robot2019.autonomous.options.StartingPosition;
import com.first1444.frc.robot2019.deepspace.FieldDimensions;
import com.first1444.frc.robot2019.deepspace.GamePieceType;
import com.first1444.frc.robot2019.deepspace.SlotLevel;
import com.first1444.frc.robot2019.subsystems.Lift;
import com.first1444.sim.api.MathUtil;
import com.first1444.sim.api.Rotation2;
import me.retrodaredevil.action.Action;
import me.retrodaredevil.action.ActionQueue;
import me.retrodaredevil.action.Actions;

import static com.first1444.sim.api.MeasureUtil.inchesToMeters;
import static java.util.Objects.requireNonNull;

@Deprecated
public class OriginalAutonomousModeCreator implements AutonomousModeCreator {
    private static final Rotation2 SIDE_CARGO_SHIP_LONG_DISTANCE_ANGLE = Rotation2.fromDegrees(4.5);
    private final OriginalAutonActionCreator actionCreator;
    private final RobotDimensions dimensions;

    public OriginalAutonomousModeCreator(OriginalAutonActionCreator actionCreator, RobotDimensions dimensions) {
        this.actionCreator = actionCreator;
        this.dimensions = dimensions;
    }

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
    public Action createAction(
            final AutonomousType autonomousType,
            final StartingPosition startingPosition,
            final GamePieceType gamePieceType,
            final SlotLevel slotLevel,
            final LineUpType lineUpType,
            final AfterComplete afterComplete,
            final Rotation2 startingOrientation){
        requireNonNull(autonomousType); requireNonNull(lineUpType); requireNonNull(startingOrientation);

        final ActionQueue actionQueue = new Actions.ActionQueueBuilder()
                .canRecycle(false)
                .canBeDone(true)
                .immediatelyDoNextWhenDone(true) // once an action is finished, do the next one immediately
                .build();

        if(gamePieceType == GamePieceType.HATCH){
            actionQueue.add(Actions.createRunOnce(actionCreator.getOperatorCreator().createGrabHatch()));
        }

        if (autonomousType == AutonomousType.DO_NOTHING) {
            if(startingPosition != null || gamePieceType != null || slotLevel != null)
                throw new IllegalArgumentException("All should be null! startingPosition: " + startingPosition + " gamePieceType: " + gamePieceType + " slotLevel" + slotLevel);
            if(lineUpType != LineUpType.NO_VISION)
                throw new IllegalArgumentException("lineUpType must be NO_VISION! It's: " + lineUpType);
            if(afterComplete != null)
                throw new IllegalArgumentException("afterComplete must be null! It's: " + afterComplete);
            actionQueue.add(actionCreator.getLogCreator().createLogMessageAction("Do nothing autonomous action starting and complete!"));
        } else if (autonomousType == AutonomousType.CROSS_LINE_FORWARD) {
            if(startingPosition != null || gamePieceType != null || slotLevel != null)
                throw new IllegalArgumentException("All should be null! startingPosition: " + startingPosition + " gamePieceType: " + gamePieceType + " slotLevel" + slotLevel);
            if(lineUpType != LineUpType.NO_VISION)
                throw new IllegalArgumentException("lineUpType must be NO_VISION! It's: " + lineUpType);
            if(afterComplete != null)
                throw new IllegalArgumentException("afterComplete must be null! It's: " + afterComplete);
            actionQueue.add(actionCreator.getLogCreator().createLogMessageAction("Cross line forward autonomous starting!"));
            actionQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(55), .5, Rotation2.DEG_90)); // cross line
            actionQueue.add(actionCreator.getLogCreator().createLogMessageAction("Cross line forward autonomous ending!"));
        } else if (autonomousType == AutonomousType.CROSS_LINE_SIDE) {
            if(gamePieceType != null || slotLevel != null)
                throw new IllegalArgumentException("All should be null! gamePieceType: " + gamePieceType + " slotLevel" + slotLevel);
            if(lineUpType != LineUpType.NO_VISION)
                throw new IllegalArgumentException("lineUpType must be NO_VISION! It's: " + lineUpType);
            if(afterComplete != null)
                throw new IllegalArgumentException("afterComplete must be null! It's: " + afterComplete);

            final boolean isLeft;
            if (startingPosition == StartingPosition.LEFT) {
                isLeft = true;
            } else if (startingPosition == StartingPosition.RIGHT) {
                isLeft = false;
            } else {
                throw new IllegalArgumentException("Cross Line Side doesn't support starting position: " + startingPosition);
            }
            actionQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(65), .5, isLeft ? Rotation2.DEG_180 : Rotation2.ZERO)); // go towards wall
            actionQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(55), .5, Rotation2.DEG_90)); // cross line
        } else if (autonomousType == AutonomousType.OFF_CENTER_CARGO_SHIP) {
            if (slotLevel != SlotLevel.LEVEL1) {
                throw new IllegalArgumentException("Got level: " + slotLevel + " with autonomous mode " + autonomousType);
            }
            if (gamePieceType != GamePieceType.HATCH) {
                throw new IllegalArgumentException("autonomousType: " + autonomousType + " (off center cargo ship) must use game piece hatch! It's: " + gamePieceType);
            }
            if (startingPosition == null) {
                throw new IllegalArgumentException("A left or right starting position must be selected for off center auto!");
            }
            final boolean isLeft; // this value was initially going to be used to tell which vision target to use, but I haven't got around to refactoring
            if (startingPosition == StartingPosition.MIDDLE_LEFT) {
                isLeft = true;
            } else if (startingPosition == StartingPosition.MIDDLE_RIGHT) {
                isLeft = false;
            } else {
                throw new IllegalArgumentException("Off Center Cargo Ship doesn't support starting position: " + startingPosition);
            }
            // It's about 130 inches to the cargo ship

            // go 100 inches
            actionQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(40.0), .3, Rotation2.DEG_90, startingOrientation)); // get off hab
            actionQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(30.0), .7, Rotation2.DEG_90, startingOrientation)); // drive a little
            actionQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(30.0), .3, Rotation2.DEG_90, startingOrientation)); // drive slower
            // went 100 inches
            assert gamePieceType == GamePieceType.HATCH : "always true";
            final Rotation2 faceAngle = getManipulatorOffset(GamePieceType.HATCH).rotate90(1); // face the manipulator towards the cargo ship
            if (MathUtil.minDistance(faceAngle.getDegrees(), startingOrientation.getDegrees(), 360) > 5) { // only rotate if we need to
                actionQueue.add(actionCreator.getLogCreator().createLogWarningAction("We are turning to face the target. Next time, start the robot in the correct position. faceAngle: " + faceAngle + " startingOrientation: " + startingOrientation));
                actionQueue.add(actionCreator.getDriveCreator().createTurnToOrientation(faceAngle));
                System.out.println("Creating auto mode where we have to turn to face cargo ship. Why would you make the robot start in another orientation anyway?");
            }
            // we are now turned to faceAngle
            final ActionQueue successAction = new Actions.ActionQueueBuilder(
                    actionCreator.getLogCreator().createLogMessageAction("Placed hatch on center cargo ship with vision!")
            ).immediatelyDoNextWhenDone(true).canBeDone(true).canRecycle(false).build();

            if(lineUpType == LineUpType.USE_VISION) {
                actionQueue.add(actionCreator.getVisionCreator().createCargoShipPlaceHatchUseVision(
                        successAction,
                        actionCreator.getLogCreator().createLogWarningAction("Failed to place hatch on center cargo ship with vision!")
                ));
            } else {
                // drive a total of 30 more inches
                actionQueue.add(new Actions.ActionMultiplexerBuilder(
                        actionCreator.getOperatorCreator().createExtendHatch(), // ready position
                        actionCreator.getDriveCreator().createGoStraight(inchesToMeters(10.0), .3, faceAngle, faceAngle) // go forward
                ).build());
                final double hatchExtend = dimensions.getHatchManipulatorActiveExtendDistanceMeters();
                if(hatchExtend < inchesToMeters(20)) {
                    actionQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(20) - hatchExtend, .15, faceAngle, faceAngle));
                }
                actionQueue.add(actionCreator.getOperatorCreator().createDropHatch());
            }
            if(afterComplete != null){
                actionQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(20), .3, Rotation2.DEG_270, faceAngle));
                if(afterComplete == AfterComplete.PREPARE_FOR_DEFENSE){
                    actionQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(50), .3, isLeft ? Rotation2.DEG_180 : Rotation2.ZERO, faceAngle));
                    actionQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(50), .3, Rotation2.DEG_90, faceAngle));
                } else {
                    final boolean hatch;
                    if (afterComplete == AfterComplete.GO_TO_LOADING_STATION_HATCH) {
                        hatch = true;
                    } else if (afterComplete == AfterComplete.GO_TO_LOADING_STATION_CARGO) {
                        hatch = false;
                    } else {
                        throw new IllegalArgumentException("Unsupported AfterComplete: " + afterComplete);
                    }
                    actionQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(70), .3, isLeft ? Rotation2.DEG_180 : Rotation2.ZERO, faceAngle));
                    final GamePieceType afterGamePieceType = hatch ? GamePieceType.HATCH : GamePieceType.CARGO;
                    final Rotation2 faceDirection = getManipulatorOffset(afterGamePieceType).minusDegrees(90);
                    actionQueue.add(actionCreator.getDriveCreator().createTurnToOrientation(faceDirection));
                    actionQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(100), .3, Rotation2.DEG_270, faceDirection));
                }
            }
        } else if (autonomousType == AutonomousType.SIDE_CARGO_SHIP) {
            if (gamePieceType == null) {
                throw new IllegalArgumentException("Side Cargo ship auto requires a game piece to be selected!");
            }
            if (slotLevel != SlotLevel.LEVEL1) {
                throw new IllegalArgumentException("Side Cargo ship auto only supports level 1. Got level: " + slotLevel);
            }
            final boolean isLeft;
            if (startingPosition == StartingPosition.LEFT) {
                isLeft = true;
            } else if (startingPosition == StartingPosition.RIGHT) {
                isLeft = false;
            } else {
                throw new IllegalArgumentException("Side Cargo Ship doesn't support starting position: " + startingPosition);
            }
            final Rotation2 longDistanceAngle = isLeft ? Rotation2.DEG_90.plus(SIDE_CARGO_SHIP_LONG_DISTANCE_ANGLE) : Rotation2.DEG_90.minus(SIDE_CARGO_SHIP_LONG_DISTANCE_ANGLE);

            final Rotation2 towardsCargoShipAngle = (isLeft ? Rotation2.ZERO : Rotation2.DEG_180);
            final Rotation2 faceAngle = towardsCargoShipAngle.plus(getManipulatorOffset(gamePieceType)); // face the manipulator towards the cargo ship
            Rotation2 distanceToFaceAngle = faceAngle.minus(startingOrientation); // in range [0..180]
            if(distanceToFaceAngle.getDegrees() < 0){
                distanceToFaceAngle = distanceToFaceAngle.unaryMinus();
            }

            double distance = inchesToMeters(212.8) - FieldDimensions.HAB_FLAT_DISTANCE - inchesToMeters(22); //we need to go this distance // 22 is random, but I measured it
            if(distanceToFaceAngle.getDegrees() > 135 || distanceToFaceAngle.getDegrees() < 45){ // turn 180 or turn 0
                distance += getManipulatorSideWidth(gamePieceType) / 2.0;
            } else {
                distance += getManipulatorSideDepth(gamePieceType) / 2.0;
            }
            actionQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(40.0), .3, Rotation2.DEG_90, startingOrientation));
            distance -= inchesToMeters(40);
            actionQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(115.0) / longDistanceAngle.getSin(), .7, longDistanceAngle, startingOrientation));
            distance -= inchesToMeters(115);
            actionQueue.add(actionCreator.getDriveCreator().createGoStraight(distance, .3, Rotation2.DEG_90, startingOrientation));

            if (distanceToFaceAngle.getDegrees() > 5) { // only rotate if we need to
                actionQueue.add(actionCreator.getDriveCreator().createTurnToOrientation(faceAngle));
                System.out.println("Creating a side cargo auto mode where we have to rotate! Why not just start the robot in the correct orientation?");
            }
            final ActionQueue successQueue = new Actions.ActionQueueBuilder(
                    Actions.createRunOnce(actionCreator.getLogCreator().createLogMessageAction("Successfully placed something on cargo ship. Will do more stuff based on afterComplete variable."))
            ).immediatelyDoNextWhenDone(true).canBeDone(true).canRecycle(false).build();

            if(lineUpType == LineUpType.USE_VISION) {
                if (gamePieceType == GamePieceType.HATCH) {
                    actionQueue.add(actionCreator.getVisionCreator().createCargoShipPlaceHatchUseVision(
                            Actions.createRunOnce(actionCreator.getLogCreator().createLogWarningAction("Failed to place Hatch!")),
                            successQueue
                    ));
                } else {
                    actionQueue.add(actionCreator.getVisionCreator().createCargoShipPlaceCargoUseVision(
                            Actions.createRunOnce(actionCreator.getLogCreator().createLogWarningAction("Failed to place Cargo!")),
                            successQueue
                    ));
                }
            } else {
                final Action driveAction = actionCreator.getDriveCreator().createGoStraight(inchesToMeters(20.0), .2, towardsCargoShipAngle, faceAngle);
                if(gamePieceType == GamePieceType.HATCH){
                    actionQueue.add(new Actions.ActionMultiplexerBuilder(
                            actionCreator.getOperatorCreator().createExtendHatch(),
                            driveAction // go towards cargo ship
                    ).build());
                    actionQueue.add(actionCreator.getOperatorCreator().createDropHatch());
                } else {
                    actionQueue.add(new Actions.ActionMultiplexerBuilder(
                            actionCreator.getLogCreator().createLogMessageAction("The lift will be raised to CARGO_CARGO_SHIP while we drive the 20 inches"),
                            actionCreator.getOperatorCreator().createRaiseLift(Lift.Position.CARGO_CARGO_SHIP),
                            driveAction // go towards cargo ship
                    ).forceUpdateInOrder(true).build());
                    actionQueue.add(actionCreator.getOperatorCreator().createReleaseCargo());
                }
                actionQueue.add(successQueue);
            }
            if(afterComplete != null){
                successQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(30.0), .4, towardsCargoShipAngle.unaryMinus(), faceAngle));
                if(afterComplete == AfterComplete.PREPARE_FOR_DEFENSE){
                    successQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(40.0), .3, Rotation2.DEG_90, faceAngle));
                } else {
                    final boolean hatch;
                    if (afterComplete == AfterComplete.GO_TO_LOADING_STATION_HATCH) {
                        hatch = true;
                    } else if (afterComplete == AfterComplete.GO_TO_LOADING_STATION_CARGO) {
                        hatch = false;
                    } else {
                        throw new IllegalArgumentException("Unsupported AfterComplete: " + afterComplete);
                    }
                    successQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(30), .4, towardsCargoShipAngle.unaryMinus(), faceAngle));
                    successQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(100), .4, Rotation2.DEG_270, faceAngle));
                    final GamePieceType afterGamePieceType = hatch ? GamePieceType.HATCH : GamePieceType.CARGO;
                    final Rotation2 faceDirection = getManipulatorOffset(afterGamePieceType).minusDegrees(90);
                    successQueue.add(actionCreator.getDriveCreator().createTurnToOrientation(faceDirection));
                }
            }
        } else if (autonomousType == AutonomousType.SIDE_ROCKET) {
            if(gamePieceType != GamePieceType.HATCH){
                throw new IllegalArgumentException("Side rocket only supports the hatch! Got: " + gamePieceType);
            }
            if (slotLevel == null) {
                throw new IllegalArgumentException("A Slot Level must be specified for side rocket autonomous!");
            }
            final boolean sideRocketIsLeft;
            if (startingPosition == StartingPosition.LEFT) {
                sideRocketIsLeft = true;
            } else if (startingPosition == StartingPosition.RIGHT) {
                sideRocketIsLeft = false;
            } else {
                throw new IllegalArgumentException("Side Rocket doesn't support starting position: " + startingPosition);
            }
            actionQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(69.56 + 6), .3, sideRocketIsLeft ? Rotation2.DEG_180 : Rotation2.ZERO, startingOrientation)); // the 6 is random
            actionQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(201.13 - 95.28) - FieldDimensions.HAB_LIP_DISTANCE - inchesToMeters(60), .5, Rotation2.DEG_90, startingOrientation)); // the 60 is random

            final ActionQueue successQueue = new Actions.ActionQueueBuilder(
                    Actions.createRunOnce(actionCreator.getLogCreator().createLogMessageAction("We successfully placed something on the rocket. If you choose an AfterComplete, this should do something now"))
            ).immediatelyDoNextWhenDone(true).canBeDone(true).canRecycle(false).build();

            final Rotation2 driveAngle = Rotation2.fromDegrees(90 - (sideRocketIsLeft ? -25 : 25)); // the angle to drive towards the rocket at if lined up perpendicular to slot
            final Rotation2 towardsCenter = sideRocketIsLeft ? Rotation2.ZERO : Rotation2.DEG_180;
            if(lineUpType == LineUpType.USE_VISION) {
                actionQueue.add(actionCreator.getVisionCreator().createRocketPlaceHatchUseVision(
                        slotLevel,
                        actionCreator.getLogCreator().createLogWarningAction("Failed to place " + gamePieceType + " on rocket."),
                        successQueue
                ));
                successQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(10), .3, driveAngle.rotate90(2), driveAngle));
                successQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(20), .3, towardsCenter, driveAngle));
            } else {
                actionQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(60 - 1), .3, Rotation2.DEG_90, startingOrientation)); // the 3 is random
                actionQueue.add(actionCreator.getDriveCreator().createTurnToOrientation(driveAngle));
                actionQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(30), .2, driveAngle, driveAngle));
                actionQueue.add(actionCreator.getOperatorCreator().createDropHatch());
                actionQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(10), .2, driveAngle.rotate90(2), driveAngle));
                actionQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(20), .3, towardsCenter, driveAngle));
                actionQueue.add(successQueue);
            }
            if(afterComplete != null) {
                if (afterComplete == AfterComplete.PREPARE_FOR_DEFENSE) {
                    successQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(40), .3, towardsCenter));
                    successQueue.add(actionCreator.getDriveCreator().createTurnToOrientation(Rotation2.DEG_90));
                    successQueue.add(new Actions.ActionMultiplexerBuilder(
                            actionCreator.getDriveCreator().createGoStraight(inchesToMeters(75), .3, Rotation2.DEG_90, Rotation2.DEG_90),
                            actionCreator.getOperatorCreator().createStowHatch()
                    ).build());
                } else {
                    final boolean hatch;
                    if (afterComplete == AfterComplete.GO_TO_LOADING_STATION_HATCH) {
                        hatch = true;
                    } else if (afterComplete == AfterComplete.GO_TO_LOADING_STATION_CARGO) {
                        hatch = false;
                    } else {
                        throw new IllegalArgumentException("Unsupported AfterComplete: " + afterComplete);
                    }
                    final GamePieceType afterGamePieceType = hatch ? GamePieceType.HATCH : GamePieceType.CARGO;
                    successQueue.add(actionCreator.getLogCreator().createLogMessageAction("going to go back 30 inches"));
                    successQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(30), .3, Rotation2.DEG_270));
                    successQueue.add(actionCreator.getLogCreator().createLogMessageAction("going to rotate"));
                    final Rotation2 faceDirection = Rotation2.fromDegrees(-90).plus(getManipulatorOffset(afterGamePieceType));
                    successQueue.add(actionCreator.getDriveCreator().createTurnToOrientation(faceDirection));
                    successQueue.add(actionCreator.getLogCreator().createLogMessageAction("Going to go straight"));
                    successQueue.add(actionCreator.getDriveCreator().createGoStraight(inchesToMeters(100), .4, Rotation2.DEG_270, faceDirection));
                }
            }
        } else {
            System.out.println("Doing nothing for autonomous type: " + autonomousType);
        }
        return actionQueue;

    }
    private Rotation2 getManipulatorOffset(GamePieceType gamePieceType){
        requireNonNull(gamePieceType);
        return gamePieceType == GamePieceType.HATCH
                ? dimensions.getHatchManipulatorPerspective().getOffset()
                : dimensions.getCargoManipulatorPerspective().getOffset();
    }
    private double getManipulatorSideWidth(GamePieceType gamePieceType){
        requireNonNull(gamePieceType);
        return gamePieceType == GamePieceType.HATCH
                ? dimensions.getHatchSideWidthMeters()
                : dimensions.getCargoSideWidthMeters();
    }
    private double getManipulatorSideDepth(GamePieceType gamePieceType){
        requireNonNull(gamePieceType);
        return gamePieceType == GamePieceType.HATCH
                ? dimensions.getHatchSideDepthMeters()
                : dimensions.getCargoSideDepthMeters();
    }
}
