package com.first1444.frc.robot2019.autonomous.actions;

import com.first1444.frc.robot2019.actions.TimedAction;
import com.first1444.frc.robot2019.subsystems.CargoIntake;

import java.util.Objects;
import java.util.function.Supplier;

public class TimedCargoIntake extends TimedAction {
	
	private final Supplier<CargoIntake> cargoIntakeSupplier;
	private final double speed;
	
	/**
	 * @param lastMillis The amount of time in millis for this to last
	 */
	public TimedCargoIntake(long lastMillis, Supplier<CargoIntake> cargoIntakeSupplier, double speed) {
		super(true, lastMillis);
		this.cargoIntakeSupplier = Objects.requireNonNull(cargoIntakeSupplier);
		this.speed = speed;
	}
	
	@Override
	protected void onUpdate() {
		super.onUpdate();
		final CargoIntake cargoIntake = cargoIntakeSupplier.get();
		Objects.requireNonNull(cargoIntake);
		cargoIntake.setSpeed(speed);
	}
	
	@Override
	protected void onEnd(boolean peacefullyEnded) {
		super.onEnd(peacefullyEnded);
		final CargoIntake cargoIntake = cargoIntakeSupplier.get();
		Objects.requireNonNull(cargoIntake);
		cargoIntake.setSpeed(0);
	}
}
