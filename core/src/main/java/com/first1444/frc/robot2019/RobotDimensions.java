package com.first1444.frc.robot2019;

public interface RobotDimensions {
	
	Perspective getHatchManipulatorPerspective();
	Perspective getCargoManipulatorPerspective();
	
	int getHatchCameraID();
	int getCargoCameraID();
	
	/**
	 * @return The amount the hatch manipulator extends outside the <em>bumpers</em> when it is fully out, in inches
	 */
	double getHatchManipulatorActiveExtendDistance();
	
	/** @return The width (excluding bumpers) of the side with the hatch manipulator*/
	double getHatchSideWidth();
	/** @return The width (excluding bumpers) of the side with the cargo manipulator*/
	double getCargoSideWidth();
	/** @return The depth (excluding bumpers) of the side with the hatch manipulator*/
	double getHatchSideDepth();
	/** @return The depth (excluding bumpers) of the side with the cargo manipulator*/
	double getCargoSideDepth();
}
