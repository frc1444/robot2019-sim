package com.first1444.frc.robot2019;

public interface RobotDimensions {

    Perspective getHatchManipulatorPerspective();
    Perspective getCargoManipulatorPerspective();

    int getHatchCameraID();
    int getCargoCameraID();

    /**
     * @return The amount the hatch manipulator extends outside the <em>bumpers</em> when it is fully out, in inches
     */
    double getHatchManipulatorActiveExtendDistanceMeters();

    /** @return The width (excluding bumpers) of the side with the hatch manipulator*/
    double getHatchSideWidthMeters();
    /** @return The width (excluding bumpers) of the side with the cargo manipulator*/
    double getCargoSideWidthMeters();
    /** @return The depth (excluding bumpers) of the side with the hatch manipulator*/
    double getHatchSideDepthMeters();
    /** @return The depth (excluding bumpers) of the side with the cargo manipulator*/
    double getCargoSideDepthMeters();
}
