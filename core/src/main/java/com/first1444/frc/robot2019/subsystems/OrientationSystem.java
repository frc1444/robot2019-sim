package com.first1444.frc.robot2019.subsystems;

import com.first1444.frc.robot2019.ShuffleboardMap;
import com.first1444.frc.robot2019.input.RobotInput;
import com.first1444.frc.robot2019.sensors.DefaultOrientation;
import com.first1444.frc.util.DynamicSendableChooser;
import com.first1444.sim.api.sensors.DefaultMutableOrientation;
import com.first1444.sim.api.sensors.MutableOrientation;
import com.first1444.sim.api.sensors.Orientation;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import me.retrodaredevil.action.SimpleAction;
import me.retrodaredevil.controller.input.InputPart;

import java.util.Objects;

import static java.util.Objects.requireNonNull;

public class OrientationSystem extends SimpleAction {
//	private final DynamicSendableChooser<Double> startingOrientationChooser;
	private final RobotInput robotInput;
	private final MutableOrientation orientation;
	public OrientationSystem(ShuffleboardMap shuffleboardMap, Orientation orientation, RobotInput robotInput) {
		super(false);
		requireNonNull(shuffleboardMap);
		this.orientation = new DefaultMutableOrientation(orientation);
		this.robotInput = requireNonNull(robotInput);
		
//		startingOrientationChooser = new DynamicSendableChooser<>();
//		startingOrientationChooser.setDefaultOption("forward (90)", 90.0);
//		startingOrientationChooser.addOption("right (0)", 0.0);
//		startingOrientationChooser.addOption("left (180)", 180.0);
//		startingOrientationChooser.addOption("backwards (270)", 270.0);
//		shuffleboardMap.getUserTab().add("Starting Orientation", startingOrientationChooser).withSize(2, 1).withPosition(9, 0);
	}
	@Deprecated
	public double getStartingOrientation(){
		return orientation.getOrientationDegrees();
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
				final double angle = robotInput.getResetGyroJoy().getAngle();
				orientation.setOrientationDegrees(angle);
//				startingOrientationChooser.addOption("Custom", angle);
//				startingOrientationChooser.setSelectedKey("Custom");
			}
		}
	}
}
