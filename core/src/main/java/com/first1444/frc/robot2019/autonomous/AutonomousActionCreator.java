package com.first1444.frc.robot2019.autonomous;

import com.first1444.frc.robot2019.deepspace.SlotLevel;
import com.first1444.frc.robot2019.subsystems.Lift;
import com.first1444.sim.api.Rotation2;
import com.first1444.sim.api.Vector2;
import me.retrodaredevil.action.Action;

public interface AutonomousActionCreator {
    interface DesiredRotationProvider {
        Rotation2 getDesiredRotation(Vector2 position);
    }

    Action createLogMessageAction(String message);
    Action createLogWarningAction(String message);
    Action createLogErrorAction(String message);

    Action createTurnToOrientation(Rotation2 desiredOrientation);
    Action createMoveToAbsolute(double speed, Vector2 position);
    Action createMoveToAbsolute(double speed, Vector2 position, Rotation2 faceDirection);
    Action createMoveToAbsolute(double speed, Vector2 position, DesiredRotationProvider desiredRotationProvider);

    Action createCargoShipPlaceHatchUseVision(Action failAction, Action successAction);
    Action createCargoShipPlaceCargoUseVision(Action failAction, Action successAction);

    Action createRocketPlaceHatchUseVision(SlotLevel slotLevel, Action failAction, Action successAction);
    Action createRocketPlaceCargoUseVision(SlotLevel slotLevel, Action failAction, Action successAction);

    /**
     * @return An action that is will be done once the hatch intake is in the ready position
     */
    Action createExtendHatch();
    Action createStowHatch();

    Action createDropHatch();
    Action createGrabHatch();

    Action createReleaseCargo();

    /**
     * @param position The desired position
     * @return Creates an action that's done when the lift has reached the desired position
     */
    Action createRaiseLift(Lift.Position position);
}
