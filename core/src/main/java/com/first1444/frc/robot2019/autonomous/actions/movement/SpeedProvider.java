package com.first1444.frc.robot2019.autonomous.actions.movement;

import com.first1444.sim.api.Vector2;

public interface SpeedProvider {
    double getSpeed(Vector2 currentAbsolutePosition);
}
