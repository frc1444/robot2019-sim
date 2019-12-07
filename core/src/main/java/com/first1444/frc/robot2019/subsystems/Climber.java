package com.first1444.frc.robot2019.subsystems;

import me.retrodaredevil.action.Action;

public interface Climber extends Action {
    /** @param speed The speed of the climber. A positive value raises the robot by pushing the climber down, a negative value retracts. */
    void setClimbSpeed(double speed);
    /** @param speed The speed of the wheels on the climber. A positive value makes the robot go forward. A negative value goes backwards and is usually never used. */
    void setDriveSpeed(double speed);
}
