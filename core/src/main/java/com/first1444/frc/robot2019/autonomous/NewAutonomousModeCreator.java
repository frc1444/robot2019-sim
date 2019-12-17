package com.first1444.frc.robot2019.autonomous;

import com.first1444.frc.robot2019.RobotDimensions;
import com.first1444.frc.robot2019.autonomous.creator.AutonomousActionCreator;
import com.first1444.frc.robot2019.autonomous.options.AfterComplete;
import com.first1444.frc.robot2019.autonomous.options.AutonomousType;
import com.first1444.frc.robot2019.autonomous.options.LineUpType;
import com.first1444.frc.robot2019.autonomous.options.StartingPosition;
import com.first1444.frc.robot2019.deepspace.GamePieceType;
import com.first1444.sim.api.Rotation2;
import com.first1444.sim.api.Transform2;
import com.first1444.sim.api.Vector2;
import com.first1444.sim.api.frc.implementations.deepspace.Field2019;
import com.first1444.sim.api.frc.implementations.deepspace.VisionTarget;
import me.retrodaredevil.action.Action;
import me.retrodaredevil.action.ActionQueue;
import me.retrodaredevil.action.Actions;

import static com.first1444.sim.api.MathUtil.minDistance;
import static java.util.Objects.requireNonNull;

public class NewAutonomousModeCreator implements AutonomousModeCreator {
    private final AutonomousActionCreator creator;
    private final RobotDimensions dimensions;

    public NewAutonomousModeCreator(AutonomousActionCreator creator, RobotDimensions dimensions) {
        this.creator = creator;
        this.dimensions = dimensions;
    }

    @Override
    public Action createAction(AutonomousSettings settings, Transform2 startingTransform) {
        final ActionQueue actionQueue = new Actions.ActionQueueBuilder()
                .canRecycle(false)
                .canBeDone(true)
                .immediatelyDoNextWhenDone(true) // once an action is finished, do the next one immediately
                .build();
        if(settings.getGamePieceType() == GamePieceType.HATCH){
            actionQueue.add(Actions.createRunOnce(creator.getOperatorCreator().createGrabHatch()));
        }
        AutonomousType autonomousType = requireNonNull(settings.getAutonomousType());
        if (autonomousType == AutonomousType.DO_NOTHING){
            actionQueue.add(creator.getLogCreator().createLogMessageAction("Do nothing autonomous action starting and complete!"));
        } else if(autonomousType == AutonomousType.CROSS_LINE_FORWARD){
            actionQueue.add(creator.getLogCreator().createLogMessageAction("Cross line forward autonomous starting!"));
            actionQueue.add(creator.getDriveCreator().createMoveToAbsolute(startingTransform.getPosition().withY(-4), .3)); // cross line
            actionQueue.add(creator.getLogCreator().createLogMessageAction("Cross line forward autonomous ending!"));
        } else if(autonomousType == AutonomousType.CROSS_LINE_SIDE){
            StartingPosition startingPosition = settings.getStartingPosition();
            final int sideMultiplier;
            if(startingPosition == StartingPosition.LEFT){
                sideMultiplier = -1;
            } else if(startingPosition == StartingPosition.RIGHT){
                sideMultiplier = 1;
            } else {
                throw new IllegalArgumentException("Unsupported starting position for cross line side. startingPosition=" + startingPosition);
            }
            actionQueue.add(creator.getDriveCreator().createMoveToAbsolute(startingTransform.getPosition().withX(3 * sideMultiplier), .3));
            actionQueue.add(creator.getDriveCreator().createMoveToAbsolute(new Vector2(3 * sideMultiplier, -4), .3));
        } else if(autonomousType == AutonomousType.OFF_CENTER_CARGO_SHIP){
            final VisionTarget target;
            final int sideMultiplier;
            final boolean left;
            StartingPosition startingPosition = settings.getStartingPosition();
            if(startingPosition == StartingPosition.MIDDLE){
                target = Field2019.CARGO_SHIP_CENTER_LEFT; // TODO don't use starting position to determine which side to go to
                sideMultiplier = -1;
                left = true;
            } else if(startingPosition == StartingPosition.MIDDLE_LEFT){
                target = Field2019.CARGO_SHIP_CENTER_LEFT;
                sideMultiplier = -1;
                left = false;
            } else if(startingPosition == StartingPosition.MIDDLE_RIGHT){
                target = Field2019.CARGO_SHIP_CENTER_RIGHT;
                sideMultiplier = 1;
                left = false;
            } else throw new IllegalArgumentException("Unsupported starting position for off center cargo ship. startingPosition=" + startingPosition);

            GamePieceType gamePieceType = settings.getGamePieceType();
            if(gamePieceType != GamePieceType.HATCH){
                throw new IllegalArgumentException("Only hatches are supported for off center cargo ship!");
            }
            Rotation2 faceDirection = Rotation2.DEG_90.minus(dimensions.getHatchManipulatorPerspective().getOffset());
            actionQueue.add(new Actions.ActionMultiplexerBuilder(
                    creator.getDriveCreator().createMoveToAbsolute(target.getTransform().getPosition().withY(-4.4), .3, startingTransform.getRotation()), // get closer to target
                    creator.getOperatorCreator().createGrabHatch(), // grab hatch to make sure we have control of it
                    creator.getOperatorCreator().createExtendHatch() // extend hatch because we want to place it
            ).build());
            if(minDistance(faceDirection.getDegrees(), startingTransform.getRotationDegrees(), 360.0) > 5){
                actionQueue.add(creator.getDriveCreator().createTurnToOrientation(faceDirection));
            }

            final ActionQueue successAction = new Actions.ActionQueueBuilder( // if a hatch was placed successfully, this will be ran
                    creator.getLogCreator().createLogMessageAction("Placed hatch on center cargo ship. We may do more stuff based on after complete.")
            ).immediatelyDoNextWhenDone(true).build();
            LineUpType lineUpType = settings.getLineUpType();
            if(lineUpType == LineUpType.USE_VISION){
                actionQueue.add(creator.getVisionCreator().createCargoShipPlaceHatchUseVision(
                        creator.getLogCreator().createLogWarningAction("Failed to place hatch!"),
                        new Actions.ActionQueueBuilder(
                                creator.getLogCreator().createLogMessageAction("Placed a hatch successfully using vision."),
                                successAction
                        ).immediatelyDoNextWhenDone(true).build()
                ));
            } else {
                assert lineUpType == LineUpType.NO_VISION;
                // -3.35
                actionQueue.add(creator.getDriveCreator().createMoveToAbsolute(target.getTransform().getPosition().withY(-3.35), .3, faceDirection));
                actionQueue.add(creator.getOperatorCreator().createDropHatch());
                actionQueue.add(creator.getLogCreator().createLogMessageAction("We placed the hatch without vision. Hopefully it was placed correctly!"));
                actionQueue.add(successAction);
            }
            successAction.add(new Actions.ActionMultiplexerBuilder(
                    creator.getDriveCreator().createMoveToAbsolute(target.getTransform().getPosition().withY(-3.6), .3, faceDirection), // back up a little
                    creator.getOperatorCreator().createStowHatch() // stow hatch
            ).build());
            AfterComplete afterComplete = settings.getAfterComplete();
            boolean loadingCargo = afterComplete == AfterComplete.GO_TO_HAB_CARGO_BAY;
            boolean loadingHatch = afterComplete == AfterComplete.GO_TO_LOADING_STATION_HATCH;
            if(afterComplete == AfterComplete.PREPARE_FOR_DEFENSE){
                successAction.add(creator.getDriveCreator().createMoveToAbsolute(new Vector2(2.0 * sideMultiplier, -3.6), .5, faceDirection));
                successAction.add(creator.getDriveCreator().createMoveToAbsolute(new Vector2(2.0 * sideMultiplier, -1.0), .5, faceDirection));
            } else if(loadingCargo || loadingHatch){
                successAction.add(creator.getDriveCreator().createMoveToAbsolute(new Vector2(2.0 * sideMultiplier, -3.6), .5, faceDirection));
                successAction.add(creator.getDriveCreator().createMoveToAbsolute(new Vector2(3.2 * sideMultiplier, -6.5), .5, faceDirection));
                if(loadingCargo){
                    Rotation2 cargoDirection = left ? Rotation2.fromDegrees(-45) : Rotation2.fromDegrees(-90 - 45);
                    successAction.add(creator.getDriveCreator().createTurnToOrientation(cargoDirection.minus(dimensions.getCargoManipulatorPerspective().getOffset())));
                } else {
                    //noinspection ConstantConditions
                    assert loadingHatch : "always true";
                    successAction.add(creator.getDriveCreator().createTurnToOrientation(Rotation2.DEG_270.minus(dimensions.getHatchManipulatorPerspective().getOffset())));
                }
            }
        } else if(autonomousType == AutonomousType.SIDE_CARGO_SHIP){

        } else if(autonomousType == AutonomousType.SIDE_ROCKET){

        } else {
            System.err.println("Doing nothing for autonomousType=" + autonomousType);
        }
        return actionQueue;
    }
}
