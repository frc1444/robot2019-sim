package com.first1444.frc.robot2019;

import com.first1444.sim.api.sensors.Orientation;

import static com.first1444.frc.util.MathUtil.mod;

public enum Perspective {
	ROBOT_FORWARD_CAM(90, false),
	ROBOT_RIGHT_CAM(0, false),
	ROBOT_LEFT_CAM(180, false),
	ROBOT_BACK_CAM(-90, false),
	
	DRIVER_STATION(90, true),
	/** When the jumbotron is on the right side of our driver station*/
	JUMBOTRON_ON_RIGHT(0, true),
	/** When the jumbotron is on the left side of our driver station*/
	JUMBOTRON_ON_LEFT(180, true);

	private final double forwardDirection;
	private final boolean useGyro;

	Perspective(double forwardDirection, boolean useGyro) {
		this.forwardDirection = forwardDirection;
		this.useGyro = useGyro;
	}

	/**
	 * @return The the direction relative to the field that "up" on the joystick corresponds to
	 */
	public double getForwardDirection(){
		return forwardDirection;
	}
	public boolean isUseGyro(){
		return useGyro;
	}
	/**
	 * If this certain orientation does not rely on a gyro, you may pass null.
	 * @return The amount to add to the desired direction to account for the given perspective
	 */
	public double getOffset(Double orientation){
		if(!isUseGyro()){
			return mod(getForwardDirection() - 90, 360);
		}
		if(orientation == null){
			throw new IllegalArgumentException();
		}
		return mod(getForwardDirection() - orientation, 360);
	}
	public double getOrientationOffset(Orientation orientation){
		return getOffset(orientation.getOrientationDegrees());
	}
}
