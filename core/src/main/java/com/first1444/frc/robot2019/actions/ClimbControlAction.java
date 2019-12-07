package com.first1444.frc.robot2019.actions;

import com.first1444.frc.robot2019.Robot;
import com.first1444.frc.robot2019.input.RobotInput;
import com.first1444.frc.robot2019.subsystems.Climber;
import me.retrodaredevil.action.SimpleAction;
import me.retrodaredevil.controller.input.InputPart;

public class ClimbControlAction extends SimpleAction {
    private final Robot robot;
    private final RobotInput input;
    public ClimbControlAction(Robot robot, RobotInput input) {
        super(true);
        this.robot = robot;
        this.input = input;
    }

    @Override
    protected void onStart() {
        super.onStart();
    }

    @Override
    protected void onUpdate() {
        super.onUpdate();
        Climber climber = robot.getClimber();
        final InputPart climbSpeed = input.getClimbLiftSpeed();
        if(climbSpeed.isDeadzone()){
            climber.setClimbSpeed(0);
            climber.setDriveSpeed(0);
        } else {
            final double speed = climbSpeed.getPosition();
            climber.setClimbSpeed(speed); // right now we'll just keep them the same
            climber.setDriveSpeed(speed);
        }
    }
}
