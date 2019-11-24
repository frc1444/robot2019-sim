package com.first1444.frc.robot2019;

import com.first1444.dashboard.value.BasicValue;
import com.first1444.frc.robot2019.subsystems.Lift;
import com.first1444.frc.robot2019.subsystems.swerve.ModuleConfig;
import com.first1444.frc.util.valuemap.MutableValueMap;

import java.text.DecimalFormat;
import java.util.Map;

import static com.first1444.sim.api.MeasureUtil.inchesToMeters;

public final class Constants {
	
	private Constants(){ throw new UnsupportedOperationException(); }

	public static final int LIFT_ENCODER_COUNTS = 26000; // max is 24786
	public static final Map<Lift.Position, Double> LIFT_POSITION_MAP;
	static {

		LIFT_POSITION_MAP = Map.of(
				Lift.Position.LEVEL1, 0.0,
				Lift.Position.CARGO_CARGO_SHIP, .29,
				Lift.Position.LEVEL2, .60 * (24000.0 / LIFT_ENCODER_COUNTS),
				Lift.Position.LEVEL3, 1.0
		);
	}

	public static final Map<String, BasicValue> ROBOT_PREFERENCES_PROPERTIES = Map.of("Show search box", BasicValue.FALSE);
	public static final double VISION_PACKET_VALIDITY_TIME = .5;

	// region CAN IDs
	/** ID for the Climb Drive wheels Victor SPX*/
	public static final int CLIMB_DRIVE_ID = 9;
	public static final int CLIMB_LIFT_PIVOT_ID = 10;
	public static final int BOOM_MASTER_ID = 11;
	public static final int HATCH_GRAB_ID = 12;
	public static final int HATCH_STOW_ID = 13;
	/** ID for the Victor SPX for the cargo intake*/
	public static final int CARGO_INTAKE_ID = 14;
	public static final int CARGO_PIVOT_ID = 15;
	public static final int HATCH_PIVOT_ID = 16;
	// endregion
	
	public static final int BOOM_LIMIT_DIO = 0;
	
	
	public static final boolean DEBUG = true;
	public static final DecimalFormat DECIMAL_FORMAT = new DecimalFormat(" #0.00;-#0.00");
	
	public static final int PID_INDEX = 0;
	public static final int SLOT_INDEX = 0;
	public static final int INIT_TIMEOUT = 10;
	public static final int LOOP_TIMEOUT = 3;
	public static final int LOOP_TIMEOUT_THREAD = LOOP_TIMEOUT;
	
	/** The number of encoder counts per revolution on a drive wheel on the swerve drive*/
	public static final int SWERVE_DRIVE_ENCODER_COUNTS_PER_REVOLUTION = 534;
	
	// endregion
	
	/** Conversion of CTRE units of 100 units/ms*/
	public static final int CTRE_UNIT_CONVERSION = 600;
	private static final int MAX_CIM_RPM = 5300;
	/** The maximum RPM of a drive wheel on the swerve drive*/
	public static final int MAX_SWERVE_DRIVE_RPM = MAX_CIM_RPM;
	/** Talon SRX counts every edge of the quadrature encoder, so 4 * 20 */
	public static final int CIMCODER_COUNTS_PER_REVOLUTION = 80;
	
	public enum Swerve2018 implements SwerveSetup{
		INSTANCE;
		
		@Override public int getFLDriveCAN() { return 4; }
		@Override public int getFRDriveCAN() { return 3; }
		@Override public int getRLDriveCAN() { return 2; }
		@Override public int getRRDriveCAN() { return 1; }
		
		@Override public int getFLSteerCAN() { return 8; }
		@Override public int getFRSteerCAN() { return 7; }
		@Override public int getRLSteerCAN() { return 6; }
		@Override public int getRRSteerCAN() { return 5; }
		
		@Override
		public double getWheelBase() {
			return 27.375;
		}
		
		@Override
		public double getTrackWidth() {
			return 22.25;
		}
		
		@Override
		public int getQuadCountsPerRevolution() {
			return 1657;
		}
		
		@Override
		public MutableValueMap<ModuleConfig> setupFL(MutableValueMap<ModuleConfig> config) {
			return config.setDouble(ModuleConfig.ABS_ENCODER_OFFSET, 147)
					.setDouble(ModuleConfig.MAX_ENCODER_VALUE, 899)
					.setDouble(ModuleConfig.MIN_ENCODER_VALUE, 10);
		}
		
		@Override
		public MutableValueMap<ModuleConfig> setupFR(MutableValueMap<ModuleConfig> config) {
			return config.setDouble(ModuleConfig.ABS_ENCODER_OFFSET, 705)
					.setDouble(ModuleConfig.MAX_ENCODER_VALUE, 891)
					.setDouble(ModuleConfig.MIN_ENCODER_VALUE, 12);
		}
		
		@Override
		public MutableValueMap<ModuleConfig> setupRL(MutableValueMap<ModuleConfig> config) {
			return config.setDouble(ModuleConfig.ABS_ENCODER_OFFSET, 775)
					.setDouble(ModuleConfig.MAX_ENCODER_VALUE, 872)
					.setDouble(ModuleConfig.MIN_ENCODER_VALUE, 13);
		}
		
		@Override
		public MutableValueMap<ModuleConfig> setupRR(MutableValueMap<ModuleConfig> config) {
			return config.setDouble(ModuleConfig.ABS_ENCODER_OFFSET, 604)
					.setDouble(ModuleConfig.MAX_ENCODER_VALUE, 895)
					.setDouble(ModuleConfig.MIN_ENCODER_VALUE, 9);
		}
	}
	
	public enum Swerve2019 implements SwerveSetup{
		INSTANCE;
		
		// Yup, we're using some of the same constants as last year, that doesn't mean everything will be the same, though!
		@Override public int getFLDriveCAN() { return 4; }
		@Override public int getFRDriveCAN() { return 3; }
		@Override public int getRLDriveCAN() { return 2; }
		@Override public int getRRDriveCAN() { return 1; }
		
		@Override public int getFLSteerCAN() { return 8; }
		@Override public int getFRSteerCAN() { return 7; }
		@Override public int getRLSteerCAN() { return 6; }
		@Override public int getRRSteerCAN() { return 5; }
		
		@Override
		public double getWheelBase() {
			return 22.75;
		}
		
		@Override
		public double getTrackWidth() {
			return 24;
		}
		
		@Override
		public int getQuadCountsPerRevolution() {
			return 628;
		}
		
		@Override
		public MutableValueMap<ModuleConfig> setupFL(MutableValueMap<ModuleConfig> config) {
			return config.setDouble(ModuleConfig.ABS_ENCODER_OFFSET, 786)
					.setDouble(ModuleConfig.MAX_ENCODER_VALUE, 883)
					.setDouble(ModuleConfig.MIN_ENCODER_VALUE, 10);
		}
		
		@Override
		public MutableValueMap<ModuleConfig> setupFR(MutableValueMap<ModuleConfig> config) {
			return config.setDouble(ModuleConfig.ABS_ENCODER_OFFSET, 708)
					.setDouble(ModuleConfig.MAX_ENCODER_VALUE, 891)
					.setDouble(ModuleConfig.MIN_ENCODER_VALUE, 10);
		}
		
		@Override
		public MutableValueMap<ModuleConfig> setupRL(MutableValueMap<ModuleConfig> config) {
			return config.setDouble(ModuleConfig.ABS_ENCODER_OFFSET, 381)
					.setDouble(ModuleConfig.MAX_ENCODER_VALUE, 896)
					.setDouble(ModuleConfig.MIN_ENCODER_VALUE, 10);
		}
		
		@Override
		public MutableValueMap<ModuleConfig> setupRR(MutableValueMap<ModuleConfig> config) {
			return config.setDouble(ModuleConfig.ABS_ENCODER_OFFSET, 840)
					.setDouble(ModuleConfig.MAX_ENCODER_VALUE, 858)
					.setDouble(ModuleConfig.MIN_ENCODER_VALUE, 10);
		}
	}
	
	public enum Dimensions implements RobotDimensions {
		INSTANCE;
		
		@Override
		public Perspective getHatchManipulatorPerspective() {
			return Perspective.ROBOT_FORWARD_CAM;
		}
		
		@Override
		public Perspective getCargoManipulatorPerspective() {
			return Perspective.ROBOT_BACK_CAM;
		}
		
		@Override
		public int getHatchCameraID() {
			return 0;
		}
		
		@Override
		public int getCargoCameraID() {
			return 1;
		}
		
		@Override
		public double getHatchManipulatorActiveExtendDistanceMeters() {
			return inchesToMeters(10);
		}
		
		@Override
		public double getHatchSideWidthMeters() {
			return inchesToMeters(31);
		}
		@Override public double getCargoSideWidthMeters() { return getHatchSideWidthMeters(); }
		
		@Override
		public double getHatchSideDepthMeters() {
			return inchesToMeters(28);
		}
		@Override public double getCargoSideDepthMeters() { return getHatchSideDepthMeters(); }
	}
	
}
