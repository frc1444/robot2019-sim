package com.first1444.frc.robot2019;

import com.first1444.sim.api.sensors.Orientation;

import static com.first1444.sim.api.MathUtil.mod;
import static java.lang.Math.toRadians;

public enum Perspective {
	ROBOT_FORWARD_CAM(0, false),
	ROBOT_RIGHT_CAM(-90, false),
	ROBOT_LEFT_CAM(90, false),
	ROBOT_BACK_CAM(-180, false),
	
	DRIVER_STATION(0, true),
	/** When the jumbotron is on the right side of our driver station*/
	JUMBOTRON_ON_RIGHT(-90, true),
	/** When the jumbotron is on the left side of our driver station*/
	JUMBOTRON_ON_LEFT(90, true);

	private final double offsetDegrees;
	private final boolean useGyro;

	Perspective(double offsetDegrees, boolean useGyro) {
		this.offsetDegrees = offsetDegrees;
		this.useGyro = useGyro;
	}
	public double getOffsetDegrees(){
		return offsetDegrees;
	}
	public double getOffsetRadians(){
		return toRadians(offsetDegrees);
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
