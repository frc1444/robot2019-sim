package com.first1444.frc.robot2019.autonomous.actions;

import com.first1444.frc.robot2019.actions.TimedAction;
import com.first1444.frc.robot2019.subsystems.CargoIntake;
import com.first1444.sim.api.Clock;

import java.util.Objects;
import java.util.function.Supplier;

import static java.util.Objects.requireNonNull;

public class TimedCargoIntake extends TimedAction {

    private final CargoIntake cargoIntake;
    private final double speed;

    public TimedCargoIntake(Clock clock, double lastSeconds, CargoIntake cargoIntake, double speed) {
        super(true, clock, lastSeconds);
        this.cargoIntake = requireNonNull(cargoIntake);
        this.speed = speed;
    }

    @Override
    protected void onUpdate() {
        super.onUpdate();
        cargoIntake.setSpeed(speed);
    }

    @Override
    protected void onEnd(boolean peacefullyEnded) {
        super.onEnd(peacefullyEnded);
        cargoIntake.setSpeed(0);
    }
}
