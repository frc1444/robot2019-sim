package com.first1444.frc.robot2019.vision;

/**
 * NOTE: Distances are in inches, angles are in degrees
 */
@Deprecated
public interface VisionPacket {
    /** @return The x position of the robot relative to the vision target (left and right)*/
    double getRobotX();
    /** @return The y position of the robot relative to the vision target (up and down)*/
    @Deprecated
    double getRobotY();
    /** @return The z position of the robot relative to the vision target (depth). This should be negative*/
    double getRobotZ();

    /** @return the x position of the vision relative to the robot where the camera's perspective is forward*/
    double getVisionX();
    /** @return the y position of the vision relative to the robot where the camera's perspective is forward*/
    @Deprecated
    double getVisionY();
    /** @return the z position of the vision relative to the robot where the camera's perspective is forward. Should be positive*/
    double getVisionZ();

    /** @return Same as {@link Math#toDegrees(double) Math.toDegrees}({@link Math#hypot(double, double) Math.hypot}({@link #getVisionX()}, {@link #getVisionZ()}))*/
    double getGroundDistance();
    /** @return the angle from the robot to the vision where the camera's perspective is forward. In degrees.
     * Same as {@link Math#atan2(double, double) Math.atan2}({@link #getVisionZ()}, {@link #getVisionX()})*/
    double getGroundAngle();


    /** @return the yaw of the vision target relative to the robot*/
    double getVisionYaw();
    /** @return the pitch of the vision target relative to the robot*/
    double getVisionPitch();
    /** @return the roll of the vision target relative to the robot*/
    double getVisionRoll();

    /** @return The x position of the target on the camera. In range [-1..1]*/
    double getImageX();
    /** @return The y position of the target on the camera. In range [-1..1]*/
    double getImageY();
}
