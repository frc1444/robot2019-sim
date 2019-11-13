package com.first1444.frc.robot2019.autonomous.actions.vision;

import com.first1444.frc.robot2019.subsystems.swerve.SwerveDistanceTracker;
import com.first1444.sim.api.drivetrain.swerve.SwerveDrive;

import java.util.Objects;

public class VisionView {
	/** The direction to the target in relation to the orientation. In degrees*/
	private final double directionToTarget;
	/** The distance to the target in inches. */
	private final double distanceToTarget;
	
	/** Orientation of the vision in relation to the orientation. In degrees*/
	private final double targetOrientation;
	
	private final SwerveDistanceTracker tracker;
	
	VisionView(double directionToTarget, double distanceToTarget, double targetOrientation, SwerveDrive swerveDrive) {
		this.directionToTarget = directionToTarget;
		this.distanceToTarget = distanceToTarget;
		this.targetOrientation = targetOrientation;
		Objects.requireNonNull(swerveDrive);
		this.tracker = new SwerveDistanceTracker(swerveDrive);
	}
	
	public SwerveDistanceTracker getTracker() {
		return tracker;
	}
	
	public double getTargetOrientation() {
		return targetOrientation;
	}
	
	public double getDistanceToTarget() {
		return distanceToTarget;
	}
	
	public double getDirectionToTarget() {
		return directionToTarget;
	}
}
