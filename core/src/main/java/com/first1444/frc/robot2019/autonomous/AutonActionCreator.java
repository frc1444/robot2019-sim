package com.first1444.frc.robot2019.autonomous;

import com.first1444.frc.robot2019.deepspace.SlotLevel;
import com.first1444.frc.robot2019.subsystems.Lift;
import me.retrodaredevil.action.Action;
import me.retrodaredevil.action.LinkedAction;
import me.retrodaredevil.action.WhenDone;

/**
 * NOTE: Each returned action will probably not be a {@link LinkedAction} so you do not have to
 * use {@link me.retrodaredevil.action.Actions#createLinkedActionRunner(Action, WhenDone, boolean)} to wrap them.
 * Doing so is not recommended and most if not all of the time, it will do nothing.
 * <p>
 * NOTE: For each failAction and successAction, they are allowed to be null and if they're an instanceof {@link LinkedAction},
 * they will be ran like a {@link LinkedAction}
 */
public interface AutonActionCreator {
	
	Action createLogMessageAction(String message);
	Action createLogWarningAction(String message);
	Action createLogErrorAction(String message);
	
	Action createTurnToOrientation(double desiredOrientation);
	@Deprecated
	Action createGoStraight(double distanceMeters, double speed, double angleDegrees);
	Action createGoStraight(double distanceMeters, double speed, double angleDegrees, double faceDirectionDegrees);

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
