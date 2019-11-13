package com.first1444.frc.robot2019;

import edu.wpi.first.wpilibj.interfaces.Gyro;

public class DummyGyro implements Gyro {
	private double angle;

	public DummyGyro(double angle) {
		this.angle = angle;
	}
	public void setAngle(double angle){
		this.angle = angle;
	}

	@Override
	public void calibrate() {
	}

	@Override
	public void reset() {
		System.out.println("Resetting the dummy gyro! Doing nothing!");
	}

	@Override
	public double getAngle() {
		return angle;
	}

	@Override
	public double getRate() {
		return 0;
	}

	@Override
	public void free() {
	}
	@Override
	public void close() {
	}
}
