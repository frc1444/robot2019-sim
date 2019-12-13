package com.first1444.frc.robot2019.autonomous.creator;

import com.first1444.frc.robot2019.subsystems.Lift;
import com.first1444.frc.util.autonomous.creator.LogActionCreator;
import me.retrodaredevil.action.Action;

public class TestOperatorActionCreator implements OperatorActionCreator {
    private final LogActionCreator logActionCreator;

    public TestOperatorActionCreator(LogActionCreator logActionCreator) {
        this.logActionCreator = logActionCreator;
    }

    @Override
    public Action createExtendHatch() {
        return logActionCreator.createLogMessageAction("Hatch intake going to ready position");
    }

    @Override
    public Action createStowHatch() {
        return logActionCreator.createLogMessageAction("Hatch intake going to stow");
    }

    @Override
    public Action createDropHatch() {
        return logActionCreator.createLogMessageAction("Dropping hatch!");
    }

    @Override
    public Action createGrabHatch() {
        return logActionCreator.createLogMessageAction("Grabbing hatch!");
    }

    @Override
    public Action createReleaseCargo() {
        return logActionCreator.createLogMessageAction("Released cargo!");
    }

    @Override
    public Action createRaiseLift(Lift.Position position) {
        return logActionCreator.createLogMessageAction("Raising lift to " + position);
    }
}
