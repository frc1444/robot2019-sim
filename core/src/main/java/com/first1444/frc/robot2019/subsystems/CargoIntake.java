package com.first1444.frc.robot2019.subsystems;

public interface CargoIntake {
	/**
	 * Some implementations require this to be called continuously to keep the same speed
	 * @param speed The speed of the intake. A positive value spits out, a negative value intakes.
	 */
	void setSpeed(double speed);
	
	
	void stow();
	void pickup();
}
