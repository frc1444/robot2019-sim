package com.first1444.frc.robot2019.actions;

import com.first1444.frc.robot2019.input.RobotInput;
import com.first1444.sim.api.drivetrain.swerve.SwerveDrive;
import com.first1444.sim.api.drivetrain.swerve.SwerveModule;
import me.retrodaredevil.action.SimpleAction;

import java.util.function.Supplier;

public class SwerveCalibrateAction extends SimpleAction {
	private final Supplier<SwerveDrive> driveSupplier;
	private final RobotInput input;
	public SwerveCalibrateAction(Supplier<SwerveDrive> driveSupplier, RobotInput input) {
		super(false);
		this.driveSupplier = driveSupplier;
		this.input = input;
	}
	
	@Override
	protected void onUpdate() {
		super.onUpdate();
		// TODO update this
		if(input.getSwerveQuickReverseCancel().isJustPressed()){
			for(SwerveModule module : driveSupplier.get().getDrivetrainData().getModules()){
//				module.setQuickReverseAllowed(false);
			}
		} else if(input.getSwerveQuickReverseCancel().isJustReleased()){
			for(SwerveModule module : driveSupplier.get().getDrivetrainData().getModules()){
//				module.setQuickReverseAllowed(true);
			}
		}
		if(input.getSwerveRecalibrate().isJustPressed()){
			for(SwerveModule module : driveSupplier.get().getDrivetrainData().getModules()){
//				module.recalibrate();
			}
		}
	}
}
