package com.first1444.frc.robot2019;

import com.first1444.sim.api.Rotation2;
import com.first1444.sim.api.sensors.Orientation;

import static com.first1444.sim.api.MathUtil.mod;
import static java.lang.Math.toRadians;

public enum Perspective {
	ROBOT_FORWARD_CAM(Rotation2.ZERO, false),
	ROBOT_RIGHT_CAM(Rotation2.fromDegrees(-90), false),
	ROBOT_LEFT_CAM(Rotation2.DEG_90, false),
	ROBOT_BACK_CAM(Rotation2.DEG_180, false),
	
	DRIVER_STATION(Rotation2.ZERO, true),
	/** When the jumbotron is on the right side of our driver station*/
	JUMBOTRON_ON_RIGHT(Rotation2.fromDegrees(-90), true),
	/** When the jumbotron is on the left side of our driver station*/
	JUMBOTRON_ON_LEFT(Rotation2.DEG_90, true);

	private final Rotation2 offset;
	private final boolean useGyro;

	Perspective(Rotation2 offset, boolean useGyro) {
	    this.offset = offset;
		this.useGyro = useGyro;
	}
	public double getOffsetDegrees(){
		return offset.getDegrees();
	}
	public double getOffsetRadians(){
		return offset.getRadians();
	}

	public Rotation2 getOffset(){
		return offset;
	}

	/**
	 * @return The the direction relative to the field that "up" on the joystick corresponds to
	 */
	@Deprecated
	public double getForwardDirection(){
		throw new RuntimeException();
	}
	public boolean isUseGyro(){
		return useGyro;
	}
	/**
	 * If this certain orientation does not rely on a gyro, you may pass null.
	 * @return The amount to add to the desired direction to account for the given perspective
	 */
	@Deprecated
	public double getOffset(Double orientation){
		if(!isUseGyro()){
			return mod(getForwardDirection() - 90, 360);
		}
		if(orientation == null){
			throw new IllegalArgumentException();
		}
		return mod(getForwardDirection() - orientation, 360);
	}
	@Deprecated
	public double getOrientationOffset(Orientation orientation){
		return getOffset(orientation.getOrientationDegrees());
	}
}
