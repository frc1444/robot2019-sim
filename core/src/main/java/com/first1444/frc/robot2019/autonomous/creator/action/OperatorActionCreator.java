package com.first1444.frc.robot2019.autonomous.creator.action;

import com.first1444.frc.robot2019.subsystems.Lift;
import me.retrodaredevil.action.Action;

public interface OperatorActionCreator {
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
