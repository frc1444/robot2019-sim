package com.first1444.frc.robot2019.subsystems;

import java.util.Objects;

public interface TaskSystem {

    /**
     * @return A non-null {@link Task} value representing the current task the robot is doing
     */
    Task getCurrentTask();

    /**
     * @param task The task to set the current task to. This cannot be null.
     */
    void setCurrentTask(Task task);

    default void toggleCurrentTask(){
        final Task task = getCurrentTask();
        Objects.requireNonNull(task);
        if(task == Task.HATCH){
            setCurrentTask(Task.CARGO);
        } else if(task == Task.CARGO){
            setCurrentTask(Task.HATCH);
        } else {
            throw new AssertionError();
        }
    }

    enum Task {
        HATCH, CARGO
    }
}
