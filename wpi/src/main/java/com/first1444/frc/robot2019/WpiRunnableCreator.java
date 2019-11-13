package com.first1444.frc.robot2019;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.first1444.frc.robot2019.input.InputUtil;
import com.first1444.frc.robot2019.sensors.DefaultOrientation;
import com.first1444.frc.robot2019.subsystems.swerve.ModuleConfig;
import com.first1444.frc.util.pid.PidKey;
import com.first1444.frc.util.valuemap.MutableValueMap;
import com.first1444.frc.util.valuemap.sendable.MutableValueMapSendable;
import com.first1444.sim.api.RunnableCreator;
import com.first1444.sim.api.drivetrain.swerve.FourWheelSwerveDriveData;
import com.first1444.sim.api.frc.FrcDriverStation;
import com.first1444.sim.api.frc.IterativeRobotRunnable;
import com.first1444.sim.wpi.WpiClock;
import com.first1444.sim.wpi.frc.WpiFrcDriverStation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import me.retrodaredevil.controller.output.DualShockRumble;
import me.retrodaredevil.controller.wpi.WpiInputCreator;

public class WpiRunnableCreator implements RunnableCreator {

	@Override
	public void prematureInit() {

	}
	@Override
	public Runnable createRunnable() {
	    ShuffleboardMap shuffleboardMap = new DefaultShuffleboardMap();
		FrcDriverStation driverStation = new WpiFrcDriverStation(DriverStation.getInstance());

		final MutableValueMapSendable<PidKey> drivePidSendable = new MutableValueMapSendable<>(PidKey.class);
		final MutableValueMapSendable<PidKey> steerPidSendable = new MutableValueMapSendable<>(PidKey.class);
		if(Constants.DEBUG) {
			shuffleboardMap.getDevTab().add("Drive PID", drivePidSendable).withProperties(Constants.ROBOT_PREFERENCES_PROPERTIES);
			shuffleboardMap.getDevTab().add("Steer PID", steerPidSendable).withProperties(Constants.ROBOT_PREFERENCES_PROPERTIES);
		}

		final MutableValueMap<PidKey> drivePid = drivePidSendable.getMutableValueMap();
		final MutableValueMap<PidKey> steerPid = steerPidSendable.getMutableValueMap();
		drivePid
				.setDouble(PidKey.P, 1.5)
				.setDouble(PidKey.F, 1.0)
				.setDouble(PidKey.CLOSED_RAMP_RATE, .25); // etc
		steerPid
				.setDouble(PidKey.P, 12)
				.setDouble(PidKey.I, .03);
		final SwerveSetup swerve = Constants.Swerve2019.INSTANCE;
//		final SwerveSetup swerve = Constants.Swerve2018.INSTANCE;
		final ShuffleboardTab talonDebug = shuffleboardMap.getDebugTab();
		final int quadCounts = swerve.getQuadCountsPerRevolution();
		FourWheelSwerveDriveData data = new FourWheelSwerveDriveData(
							new TalonSwerveModule("front right", swerve.getFRDriveCAN(), swerve.getFRSteerCAN(), quadCounts, drivePid, steerPid,
									swerve.setupFR(createModuleConfig(shuffleboardMap, "front right module")), talonDebug),

							new TalonSwerveModule("front left", swerve.getFLDriveCAN(), swerve.getFLSteerCAN(), quadCounts, drivePid, steerPid,
									swerve.setupFL(createModuleConfig(shuffleboardMap, "front left module")), talonDebug),

							new TalonSwerveModule("rear left", swerve.getRLDriveCAN(), swerve.getRLSteerCAN(), quadCounts, drivePid, steerPid,
									swerve.setupRL(createModuleConfig(shuffleboardMap, "rear left module")), talonDebug),

							new TalonSwerveModule("rear right", swerve.getRRDriveCAN(), swerve.getRRSteerCAN(), quadCounts, drivePid, steerPid,
									swerve.setupRR(createModuleConfig(shuffleboardMap, "rear right module")), talonDebug),
				swerve.getWheelBase(), swerve.getTrackWidth()
		);
		final var lift = new MotorLift();
		final var cargoIntake = new MotorCargoIntake(new VictorSPX(Constants.CARGO_INTAKE_ID), new TalonSRX(Constants.CARGO_PIVOT_ID));
		final var hatchIntake = new MotorHatchIntake(new TalonSRX(Constants.HATCH_GRAB_ID), new TalonSRX(Constants.HATCH_STOW_ID), new TalonSRX(Constants.HATCH_PIVOT_ID));
		final var climber = new MotorClimber(new TalonSRX(Constants.CLIMB_LIFT_PIVOT_ID), new VictorSPX(Constants.CLIMB_DRIVE_ID));

		final BNO055 gyro = new BNO055();
		gyro.SetMode(BNO055.IMUMode.NDOF);

		final Robot[] robotReference = {null};
		Robot robot = new Robot(
				driverStation, new WpiClock(), shuffleboardMap,
				InputUtil.createPS4Controller(new WpiInputCreator(0)), new WpiInputCreator(1), new WpiInputCreator(2), new DualShockRumble(new WpiInputCreator(5).createRumble()),
				new DefaultOrientation(gyro, () -> 0.0, true),
				data, lift, cargoIntake, hatchIntake, climber,
				new CameraSystem(shuffleboardMap, () -> robotReference[0].getTaskSystem())
		);
		robotReference[0] = robot;
		return new IterativeRobotRunnable(robot, driverStation);
	}
	private MutableValueMap<ModuleConfig> createModuleConfig(ShuffleboardMap shuffleboardMap, String name){
		final MutableValueMapSendable<ModuleConfig> config = new MutableValueMapSendable<>(ModuleConfig.class);
		if(Constants.DEBUG) {
			shuffleboardMap.getDevTab().add(name, config).withProperties(Constants.ROBOT_PREFERENCES_PROPERTIES);
		}
		return config.getMutableValueMap();
	}

}
