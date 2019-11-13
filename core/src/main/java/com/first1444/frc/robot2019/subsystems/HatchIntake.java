package com.first1444.frc.robot2019.subsystems;

public interface HatchIntake {
	void hold();
	void drop();
	void neutralHold();

	void groundPosition();
	void readyPosition();
	void stowedPosition();
	
	boolean isDesiredPositionReached(); // TODO create an action that actually uses this
	
}
