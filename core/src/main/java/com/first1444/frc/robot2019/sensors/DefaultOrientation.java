package com.first1444.frc.robot2019.sensors;

import com.first1444.sim.api.sensors.Orientation;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import java.util.Objects;
import java.util.function.Supplier;

import static com.first1444.sim.api.MathUtil.mod;
import static java.lang.Math.toRadians;

public class DefaultOrientation implements Orientation {

	private final Gyro gyro;
	private final boolean isGyroReversed;

	/**
	 * This expects that when the angle of the gyro
	 * @param gyro The gyro representing the offset from when it was reset.:
	 * @param isGyroReversed true if a negative value represents counter-clockwise and if a positive value represents clockwise,
	 *                       if a negative value represents clockwise and a positive value represents counter-clockwise.
	 *                       This normally has to be true because of how WPILib does most things.
	 */
	public DefaultOrientation(Gyro gyro, boolean isGyroReversed) {
		this.gyro = gyro;
		this.isGyroReversed = isGyroReversed;
	}
	public DefaultOrientation(Gyro gyro){
		this(gyro, true);
	}

	@Override
	public double getOrientationDegrees() {
		return gyro.getAngle() * (isGyroReversed ? -1 : 1); // TODO decide if we're OK with it going over 360 and under 0
	}

	@Override
	public double getOrientationRadians() {
		return toRadians(getOrientationDegrees());
	}
}
