package com.first1444.frc.robot2019.subsystems;

import me.retrodaredevil.action.Action;

public interface HatchIntake extends Action {
    void hold();
    void drop();
    void neutralHold();

    void readyPosition();
    void stowedPosition();

    boolean isDesiredPositionReached(); // notTO DO create an action that actually uses this - never mind. No need for this

}
