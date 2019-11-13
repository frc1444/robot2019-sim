package com.first1444.frc.robot2019.subsystems;

import com.first1444.frc.robot2019.ShuffleboardMap;
import com.first1444.frc.robot2019.input.RobotInput;
import com.first1444.frc.robot2019.sensors.DefaultOrientation;
import com.first1444.frc.util.DynamicSendableChooser;
import com.first1444.sim.api.sensors.Orientation;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import me.retrodaredevil.action.SimpleAction;
import me.retrodaredevil.controller.input.InputPart;

import java.util.Objects;

public class OrientationSystem extends SimpleAction {
	private final DynamicSendableChooser<Double> startingOrientationChooser;
	private final Gyro gyro;
	private final RobotInput robotInput;
	private final Orientation orientation; // TODO make MutableOrientation
	public OrientationSystem(ShuffleboardMap shuffleboardMap, Gyro gyro, RobotInput robotInput) {
		super(false);
		Objects.requireNonNull(shuffleboardMap);
		this.gyro = Objects.requireNonNull(gyro);
		this.robotInput = Objects.requireNonNull(robotInput);
		
		startingOrientationChooser = new DynamicSendableChooser<>();
		startingOrientationChooser.setDefaultOption("forward (90)", 90.0);
		startingOrientationChooser.addOption("right (0)", 0.0);
		startingOrientationChooser.addOption("left (180)", 180.0);
		startingOrientationChooser.addOption("backwards (270)", 270.0);
		shuffleboardMap.getUserTab().add("Starting Orientation", startingOrientationChooser).withSize(2, 1).withPosition(9, 0);
		
		orientation = new DefaultOrientation(gyro, this::getStartingOrientation);
	}
	public double getStartingOrientation(){
		return startingOrientationChooser.getSelected();
	}
	public void resetGyro(){
		gyro.reset();
	}
	public Orientation getOrientation(){
		return orientation;
	}
	
	@Override
	protected void onUpdate() {
		super.onUpdate();
		{ // resetting the gyro code
			final InputPart x = robotInput.getResetGyroJoy().getXAxis();
			final InputPart y = robotInput.getResetGyroJoy().getYAxis();
			if (x.isDown() || y.isDown()){
				gyro.reset();
				final double angle = robotInput.getResetGyroJoy().getAngle();
				startingOrientationChooser.addOption("Custom", angle);
				startingOrientationChooser.setSelectedKey("Custom");
			}
		}
	}
}
