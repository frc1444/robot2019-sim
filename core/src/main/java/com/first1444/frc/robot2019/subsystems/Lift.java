package com.first1444.frc.robot2019.subsystems;


import me.retrodaredevil.action.Action;

public interface Lift extends Action {
	void setDesiredPosition(double desiredPosition);
	void setDesiredPosition(Position desiredPosition);
	
	/**
	 *
	 * @return true if the position set with {@link #setDesiredPosition(double)} or {@link #setDesiredPosition(Position)} is reached, false otherwise
	 */
	boolean isDesiredPositionReached();
	/** Sets the manual speed of the lift.*/
	void setManualSpeed(double speed, boolean overrideSpeedSafety);
	void lockCurrentPosition();
	
	/**
	 * null can represent being disabled, or neutral.
	 * @return The {@link LiftMode} or null.
	 */
	LiftMode getLiftMode();
	
	enum LiftMode {
		SPEED, POSITION
	}
	
	
	enum Position {
		LEVEL1,
		LEVEL2,
		LEVEL3,
		CARGO_CARGO_SHIP
	}
	
}
