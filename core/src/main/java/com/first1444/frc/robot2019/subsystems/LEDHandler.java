package com.first1444.frc.robot2019.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import me.retrodaredevil.action.SimpleAction;

/**
 * This handles updating the LED and should never be ended
 */
public class LEDHandler extends SimpleAction {
	private final RobotBase robot;
	public LEDHandler(RobotBase robot) { // change to Robot if in the future if we need to
		super(false);
		this.robot = robot;
	}

	@Override
	protected void onUpdate() {
		super.onUpdate();
		if(robot.isDisabled()){ // disabled LED
			disabledLED();
		} else if(robot.isAutonomous() || robot.isTest()){ // autonomous LED
			autoLED();
		} else if(robot.isOperatorControl()){ // teleop LED
			teleopLED();
		}
	}
	private void disabledLED(){
	}
	private void autoLED(){

	}
	private void teleopLED(){
		final double timeLeft = DriverStation.getInstance().getMatchTime(); // in range [0, 135]
		if(timeLeft > 130){ // flash for 5 seconds when starting

		} else if(timeLeft < 15){ // countdown

		} else { // during teleop

		}
	}
}
