package com.first1444.frc.robot2019.subsystems;

import com.first1444.frc.robot2019.ShuffleboardMap;
import com.first1444.frc.robot2019.input.RobotInput;
import com.first1444.sim.api.sensors.DefaultMutableOrientation;
import com.first1444.sim.api.sensors.MutableOrientation;
import com.first1444.sim.api.sensors.Orientation;
import me.retrodaredevil.action.SimpleAction;
import me.retrodaredevil.controller.input.InputPart;

import static java.util.Objects.requireNonNull;

public class OrientationSystem extends SimpleAction {
	private final RobotInput robotInput;
	private final MutableOrientation orientation;
	public OrientationSystem(ShuffleboardMap shuffleboardMap, Orientation orientation, RobotInput robotInput) {
		super(false);
		requireNonNull(shuffleboardMap);
		this.orientation = new DefaultMutableOrientation(orientation);
		this.robotInput = requireNonNull(robotInput);
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
			}
		}
	}
}
