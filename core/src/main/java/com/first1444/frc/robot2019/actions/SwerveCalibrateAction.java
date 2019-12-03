package com.first1444.frc.robot2019.actions;

import com.first1444.frc.robot2019.input.RobotInput;
import com.first1444.sim.api.drivetrain.swerve.SwerveDrive;
import com.first1444.sim.api.drivetrain.swerve.SwerveModule;
import me.retrodaredevil.action.SimpleAction;

public class SwerveCalibrateAction extends SimpleAction {
    private final SwerveDrive drive;
	private final RobotInput input;
	public SwerveCalibrateAction(SwerveDrive drive, RobotInput input) {
		super(false);
		this.drive = drive;
		this.input = input;
	}
	
	@Override
	protected void onUpdate() {
		super.onUpdate();
		// TODO update this
		if(input.getSwerveQuickReverseCancel().isJustPressed()){
			for(SwerveModule module : drive.getDrivetrainData().getModules()){
//				module.setQuickReverseAllowed(false);
			}
		} else if(input.getSwerveQuickReverseCancel().isJustReleased()){
			for(SwerveModule module : drive.getDrivetrainData().getModules()){
//				module.setQuickReverseAllowed(true);
			}
		}
		if(input.getSwerveRecalibrate().isJustPressed()){
			for(SwerveModule module : drive.getDrivetrainData().getModules()){
//				module.recalibrate();
			}
		}
	}
}
