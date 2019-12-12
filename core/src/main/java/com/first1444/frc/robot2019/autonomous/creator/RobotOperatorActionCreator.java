package com.first1444.frc.robot2019.autonomous.creator;

import com.first1444.frc.robot2019.Robot;
import com.first1444.frc.robot2019.autonomous.actions.HatchIntakeAction;
import com.first1444.frc.robot2019.autonomous.actions.HatchPositionAction;
import com.first1444.frc.robot2019.autonomous.actions.RaiseLift;
import com.first1444.frc.robot2019.autonomous.actions.TimedCargoIntake;
import com.first1444.frc.robot2019.subsystems.Lift;
import me.retrodaredevil.action.Action;

public class RobotOperatorActionCreator implements OperatorActionCreator {
    private final Robot robot;

    public RobotOperatorActionCreator(Robot robot) {
        this.robot = robot;
    }

    @Override
    public Action createExtendHatch() {
        return HatchPositionAction.createReady(robot.getHatchIntake());
    }

    @Override
    public Action createStowHatch() {
        return HatchPositionAction.createStow(robot.getHatchIntake());
    }

    @Override
    public Action createDropHatch() {
        return HatchIntakeAction.createDrop(robot.getClock(), robot.getHatchIntake());
    }

    @Override
    public Action createGrabHatch() {
        return HatchIntakeAction.createGrab(robot.getClock(), robot.getHatchIntake());
    }

    @Override
    public Action createReleaseCargo() {
        return new TimedCargoIntake(robot.getClock(), .5, robot.getCargoIntake(), 1);
    }

    @Override
    public Action createRaiseLift(Lift.Position position) {
        return new RaiseLift(robot.getLift(), position);
    }
}
