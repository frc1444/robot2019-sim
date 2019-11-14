package com.first1444.frc.robot2019.sensors;

import com.first1444.sim.api.sensors.Orientation;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import java.util.Objects;
import java.util.function.Supplier;

import static com.first1444.sim.api.MathUtil.mod;
import static java.lang.Math.toRadians;

public class DefaultOrientation implements Orientation {

	private final Gyro gyro;
	/** The supplier for the starting orientation of the robot or the orientation when the gyro is reset. Normally 90 */
	private final Supplier<Double> startingOrientationSupplier;
	private final boolean isGyroReversed;

	/**
	 * This expects that when the angle of the gyro
	 * @param gyro The gyro representing the offset from when it was reset.:
	 * @param startingOrientationSupplier The supplier for the starting orientation. 0=right, 90=forward, 180=left, 270=backwards
	 * @param isGyroReversed true if a negative value represents counter-clockwise and if a positive value represents clockwise,
	 *                       if a negative value represents clockwise and a positive value represents counter-clockwise.
	 *                       This normally has to be true because of how WPILib does most things.
	 */
	public DefaultOrientation(Gyro gyro, Supplier<Double> startingOrientationSupplier, boolean isGyroReversed) {
		this.gyro = gyro;
		this.startingOrientationSupplier = Objects.requireNonNull(startingOrientationSupplier);
		this.isGyroReversed = isGyroReversed;
	}
	public DefaultOrientation(Gyro gyro, Supplier<Double> startingOrientationSupplier){
		this(gyro, startingOrientationSupplier, true);
	}

	@Override
	public double getOrientationDegrees() {
		final double angle = gyro.getAngle() * (isGyroReversed ? -1 : 1); // how much the robot has turned since starting rotation
		final double r = startingOrientationSupplier.get() + angle;
		return mod(r, 360);
	}

	@Override
	public double getOrientationRadians() {
		return toRadians(getOrientationDegrees());
	}
}
