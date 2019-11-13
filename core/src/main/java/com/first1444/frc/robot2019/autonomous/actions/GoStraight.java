package com.first1444.frc.robot2019.autonomous.actions;

import com.first1444.frc.robot2019.subsystems.swerve.SwerveDistanceTracker;
import com.first1444.frc.util.MathUtil;
import com.first1444.sim.api.Vector2;
import com.first1444.sim.api.drivetrain.swerve.SwerveDrive;
import com.first1444.sim.api.sensors.Orientation;
import me.retrodaredevil.action.SimpleAction;

import java.util.function.Supplier;

import static java.lang.Math.*;

public class GoStraight extends SimpleAction {
	private final double distance;
	private final double speed;
//	private final double directionDegrees;
	private final double x, y;
	private final Double faceDirection;
	private final Supplier<SwerveDrive> driveSupplier;
	private final Supplier<Orientation> orientationSupplier;
	
	private SwerveDistanceTracker tracker = null;


	public GoStraight(double distance, double speed, double x, double y, Double faceDirectionDegrees,
					  Supplier<SwerveDrive> driveSupplier, Supplier<Orientation> orientationSupplier) {
		super(true);
		this.distance = distance;
		this.speed = speed;
		this.x = x;
		this.y = y;
		this.faceDirection = faceDirectionDegrees;
		this.driveSupplier = driveSupplier;
		this.orientationSupplier = orientationSupplier;
	}
	public static GoStraight createGoStraightAtHeading(double distance, double speed, double headingDegrees, Double faceDirectionDegrees,
													   Supplier<SwerveDrive> driveSupplier, Supplier<Orientation> orientationSupplier){
		final double radians = toRadians(headingDegrees);
		return new GoStraight(distance, speed, cos(radians), sin(radians), faceDirectionDegrees, driveSupplier, orientationSupplier);
	}
	
	@Override
	protected void onStart() {
		super.onStart();
		tracker = new SwerveDistanceTracker(driveSupplier.get());
	}
	
	@Override
	protected void onUpdate() {
		super.onUpdate();

		final SwerveDrive drive = tracker.getDrive();
		final double minChange;
		final Orientation orientation = orientationSupplier.get();
		final double currentOrientation = orientation.getOrientationDegrees();
		if(faceDirection != null) {
			minChange = MathUtil.minChange(faceDirection, currentOrientation, 360);
		} else {
			minChange = 0;
		}
		final double turnAmount = .75 * max(-1, min(1, minChange / -40));
		drive.setControl(new Vector2(x * speed, y * speed).rotateDegrees(currentOrientation), turnAmount, 1); // TODO check this

		setDone(tracker.calculateDistance() >= distance);

	}
}
