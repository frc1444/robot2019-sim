package com.first1444.frc.robot2019.autonomous.original;

import com.first1444.frc.robot2019.Constants;
import com.first1444.frc.robot2019.deepspace.SlotLevel;
import com.first1444.frc.robot2019.subsystems.Lift;
import com.first1444.sim.api.Rotation2;
import me.retrodaredevil.action.Action;
import me.retrodaredevil.action.Actions;
import me.retrodaredevil.action.WhenDone;

import java.io.PrintStream;

public class TestOriginalAutonActionCreator implements OriginalAutonActionCreator {
    private final PrintStream out;

    public TestOriginalAutonActionCreator(PrintStream out) {
        this.out = out;
    }
    public TestOriginalAutonActionCreator(){
        this(System.out);
    }

    @Override
    public Action createLogMessageAction(String message) {
        return Actions.createRunOnce(() -> out.println("[INFO]:  " + message));
    }

    @Override
    public Action createLogWarningAction(String message) {
        return Actions.createRunOnce(() -> out.println("[WARN]:  " + message));
    }
    @Override
    public Action createLogErrorAction(String message) {
        return Actions.createRunOnce(() -> out.println("[ERROR]: " + message));
    }

    @Override
    public Action createTurnToOrientation(Rotation2 desiredOrientation) {
        return createLogMessageAction("Turning to orientation: " + desiredOrientation);
    }

    @Override
    public Action createGoStraight(double distanceMeters, double speed, Rotation2 angle) {
        return createLogMessageAction("Going straight for " + distanceMeters + " meters at " + Constants.DECIMAL_FORMAT.format(speed)
                + " with " + angle + " heading.");
    }

    @Override
    public Action createGoStraight(double distanceMeters, double speed, Rotation2 angle, Rotation2 faceDirection) {
        return createLogMessageAction("Going straight for " + distanceMeters + " meters at " + Constants.DECIMAL_FORMAT.format(speed)
                + " with " + angle + " heading while facing " + faceDirection);
    }

    @Override
    public Action createCargoShipPlaceHatchUseVision(Action failAction, Action successAction) {
        return Actions.createLinkedActionRunner(
                Actions.createLinkedAction(createLogMessageAction("Placing hatch at cargo ship using vision"), successAction),
                WhenDone.CLEAR_ACTIVE_AND_BE_DONE, true
        );
    }

    @Override
    public Action createCargoShipPlaceCargoUseVision(Action failAction, Action successAction) {
        return Actions.createLinkedActionRunner(
                Actions.createLinkedAction(createLogMessageAction("Placing cargo at cargo ship using vision"), successAction),
                WhenDone.CLEAR_ACTIVE_AND_BE_DONE, true
        );
    }

    @Override
    public Action createRocketPlaceCargoUseVision(SlotLevel slotLevel, Action failAction, Action successAction) {
        return Actions.createLinkedActionRunner(
                Actions.createLinkedAction(createLogMessageAction("Placing cargo on rocket at " + slotLevel + " using vision"), successAction),
                WhenDone.CLEAR_ACTIVE_AND_BE_DONE, true
        );
    }

    @Override
    public Action createRocketPlaceHatchUseVision(SlotLevel slotLevel, Action failAction, Action successAction) {
        return Actions.createLinkedActionRunner(
                Actions.createLinkedAction(createLogMessageAction("Placing hatch on rocket at " + slotLevel + " using vision"), successAction),
                WhenDone.CLEAR_ACTIVE_AND_BE_DONE, true
        );
    }

    @Override
    public Action createExtendHatch() {
        return createLogMessageAction("Hatch intake going to ready position");
    }

    @Override
    public Action createStowHatch() {
        return createLogMessageAction("Hatch intake going to stow");
    }

    @Override
    public Action createDropHatch() {
        return createLogMessageAction("Dropping hatch!");
    }

    @Override
    public Action createGrabHatch() {
        return createLogMessageAction("Grabbing hatch!");
    }

    @Override
    public Action createReleaseCargo() {
        return createLogMessageAction("Released cargo!");
    }

    @Override
    public Action createRaiseLift(Lift.Position position) {
        return createLogMessageAction("Raising lift to " + position);
    }
}
