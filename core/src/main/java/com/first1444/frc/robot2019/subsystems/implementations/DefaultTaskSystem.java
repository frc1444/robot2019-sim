package com.first1444.frc.robot2019.subsystems.implementations;

import com.first1444.frc.robot2019.input.RobotInput;
import com.first1444.frc.robot2019.subsystems.TaskSystem;
import me.retrodaredevil.action.SimpleAction;

import java.util.Objects;

public class DefaultTaskSystem extends SimpleAction implements TaskSystem {
    private final RobotInput robotInput;
    private Task task = Task.HATCH;

    public DefaultTaskSystem(RobotInput robotInput) {
        super(false);
        this.robotInput = robotInput;
    }

    @Override
    protected void onUpdate() {
        super.onUpdate();
        if(robotInput.getCameraToggleButton().isJustPressed()){
            toggleCurrentTask();
        }
    }

    @Override
    public Task getCurrentTask() {
        return task;
    }

    @Override
    public void setCurrentTask(Task task) {
        this.task = Objects.requireNonNull(task);
    }
}
