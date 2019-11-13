package com.first1444.frc.robot2019.vision;

import static java.lang.Math.*;

public class ImmutableVisionPacket implements VisionPacket{
	private final double robotX, robotY, robotZ, yaw, pitch, roll, imageX, imageY;
	private final double visionX, visionY, visionZ;
	private final double groundDistance, groundAngle;
	
	/**
	 *  @param robotX The x (Left to right)
	 * @param robotY The y (Up and down)
	 * @param robotZ The z (Depth)
	 * @param yaw The yaw (rotation around the y axis) (left and right rotation)
	 * @param pitch The pitch (rotation around the x axis) (Up and down tilt)
	 * @param roll The roll (rotation around the z axis) (How much it is turned) (steering wheel style)
	 * @param imageX The x position of the vision from the perspective of the camera in range [-1..1]. A positive value is right
	 * @param imageY The y position of the vision from the perspective of the camera in range [-1..1]. A positive value is up
	 */
	public ImmutableVisionPacket(double robotX, double robotY, double robotZ, double yaw, double pitch, double roll, double imageX, double imageY) {
		if(abs(imageX) > 1)
			throw new IllegalArgumentException();
		if(abs(imageY) > 1)
			throw new IllegalArgumentException();
		this.robotX = robotX;
		this.robotY = robotY;
		this.robotZ = robotZ;
		this.yaw = yaw;
		this.pitch = pitch;
		this.roll = roll;
		this.imageX = imageX;
		this.imageY = imageY;
		
		final double yawRadians = toRadians(yaw);
		
		final double rotationRadians = yawRadians;
		final double sinRotation = sin(rotationRadians);
		final double cosRotation = cos(rotationRadians);
		
		// instead of a vector pointing backwards, we now have the direction we need to go to get to the target
		visionX = -(robotX * cosRotation - robotZ * sinRotation);
		visionY = -robotY;
		visionZ = -(robotX * sinRotation + robotZ * cosRotation);
		
		groundDistance = hypot(visionX, visionZ);
		groundAngle = toDegrees(atan2(visionZ, visionX));
	}
	
	@Override
	public double getRobotX() {
		return robotX;
	}
	
	@Override
	public double getRobotY() {
		return robotY;
	}
	
	@Override
	public double getRobotZ() {
		return robotZ;
	}
	
	@Override
	public double getVisionX() {
		return visionX;
	}
	
	@Override
	public double getVisionY() {
		return visionY;
	}
	
	@Override
	public double getVisionZ() {
		return visionZ;
	}
	
	@Override
	public double getGroundDistance() {
		return groundDistance;
	}
	
	@Override
	public double getGroundAngle() {
		return groundAngle;
	}
	
	@Override
	public double getVisionYaw() {
		return yaw;
	}
	
	@Override
	public double getVisionPitch() {
		return pitch;
	}
	
	@Override
	public double getVisionRoll() {
		return roll;
	}
	
	@Override
	public double getImageX() {
		return imageX;
	}
	
	@Override
	public double getImageY() {
		return imageY;
	}
	
	@Override
	public String toString() {
		return "ImmutableVisionPacket{" +
				"x=" + robotX +
				", y=" + robotY +
				", z=" + robotZ +
				", yaw=" + yaw +
				", pitch=" + pitch +
				", roll=" + roll +
				", groundDistance=" + groundDistance +
				", groundAngle=" + groundAngle +
				'}';
	}
	
}
