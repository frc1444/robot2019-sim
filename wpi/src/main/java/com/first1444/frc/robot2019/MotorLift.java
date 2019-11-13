package com.first1444.frc.robot2019;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.first1444.frc.robot2019.subsystems.Lift;
import edu.wpi.first.wpilibj.DigitalInput;
import me.retrodaredevil.action.SimpleAction;

import java.util.Objects;
import java.util.function.BooleanSupplier;

import static java.lang.Math.*;

public class MotorLift extends SimpleAction implements Lift {
//	private static final int ENCODER_COUNTS = 24000; // max is 24786
//	private static final int ENCODER_COUNTS = 24786; // max is 24786
	private static final double LOW_POSITION_SCALE_START = .35;
	private static final double HIGH_POSITION_SCALE_START = .9;
	private static final double DESIRED_REACHED_POSITION_DEADZONE = .1;
	private static final TalonSRXConfiguration MASTER_CONFIG;

	static {
		MASTER_CONFIG = new TalonSRXConfiguration();
		
		// forward limit switch
		MASTER_CONFIG.forwardSoftLimitEnable = true;
		MASTER_CONFIG.forwardSoftLimitThreshold = 24786;
		
		MASTER_CONFIG.reverseSoftLimitEnable = true;
		MASTER_CONFIG.reverseSoftLimitThreshold = 1000;
		
		
	}
	
	private final TalonSRX master;
	private final BooleanSupplier limitDown;
	
	private LiftMode mode = null;
	/** The control based on {@link #mode}. null represents lock*/
	private Double control = 0.0;
	private boolean overrideSpeedSafety = false;
	
	private boolean desiredPositionReached = false;
	
	public MotorLift() {
		super(true);
		master = new TalonSRX(Constants.BOOM_MASTER_ID);
		final DigitalInput digitalInput = new DigitalInput(Constants.BOOM_LIMIT_DIO);
		
		/*
		For whatever reason, when the signal and ground wires aren't connected, or there's nothing connected to the
		DIO port, a value of true is returned. This is the opposite of what you expect which is weird. This means that
		if I do !digitalInput.get(), it's normally open and if I do digitalInput.get(), it's normally closed.
		 */
		limitDown = () -> !digitalInput.get(); // normally open
		CTREUtil.reportError(
				(errorCode, index) -> {
					if(errorCode != ErrorCode.OK) {
						System.err.println("Got error code: " + errorCode + " at index: " + index + " while initializing Lift!");
					}
				},
				() -> master.configFactoryDefault(Constants.INIT_TIMEOUT),
				() -> master.configAllSettings(MASTER_CONFIG, Constants.INIT_TIMEOUT),
				() -> master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.PID_INDEX, Constants.INIT_TIMEOUT),
				() -> master.config_kP(Constants.SLOT_INDEX, .13)
		);
		master.setNeutralMode(NeutralMode.Brake);
		master.setSensorPhase(true); // this needs to be called before setInverted
//		master.setInverted(InvertType.InvertMotorOutput);
	}
	private static int getEncoderCountsFromPosition(double position){
		if(position < 0 || position > 1){
			throw new IllegalArgumentException();
		}
		return (int) Math.round(position * Constants.LIFT_ENCODER_COUNTS);
	}
	private static double getPositionFromEncoderCounts(int encoderCounts){
		return encoderCounts / (double) Constants.LIFT_ENCODER_COUNTS;
	}
	public void setDesiredPosition(double desiredPosition){
		if(desiredPosition < 0 || desiredPosition > 1){
			throw new IllegalArgumentException();
		}
		if (mode != LiftMode.POSITION || control != desiredPosition) {
			desiredPositionReached = false;
		}
		mode = LiftMode.POSITION;
		control = desiredPosition;
		overrideSpeedSafety = false;
	}
	
	@Override
	public void setDesiredPosition(Position desiredPosition) {
		Objects.requireNonNull(desiredPosition);
		final double newPosition = Constants.LIFT_POSITION_MAP.get(desiredPosition);
		if (mode != LiftMode.POSITION || control != newPosition) {
			desiredPositionReached = false;
		}
		mode = LiftMode.POSITION;
		control = newPosition;
		overrideSpeedSafety = false;
	}
	
	/**
	 *
	 * @return true if the position set with {@link #setDesiredPosition(double)} or {@link #setDesiredPosition(Position)} is reached
	 */
	@Override
	public boolean isDesiredPositionReached(){
		return desiredPositionReached;
	}
	@Override
	public void setManualSpeed(double speed, boolean overrideSpeedSafety){
		if(speed < -1 || speed > 1){
			throw new IllegalArgumentException();
		}
		mode = LiftMode.SPEED;
		control = speed;
		this.overrideSpeedSafety = overrideSpeedSafety;
		desiredPositionReached = false;
	}
	
	
	@Override
	public void lockCurrentPosition() {
		mode = LiftMode.POSITION;
		control = null;
		overrideSpeedSafety = false;
		desiredPositionReached = false;
	}
	
	@Override
	public LiftMode getLiftMode() {
		return mode;
	}
	@Override
	protected void onUpdate() {
		super.onUpdate();
		
		final boolean reverseLimit = limitDown.getAsBoolean();
		Integer encoderCounts = null;
		if(reverseLimit){
			master.setSelectedSensorPosition(0);
			encoderCounts = 0;
		}
		if(control == null){ // lock position
			if(encoderCounts == null) {
				encoderCounts = master.getSelectedSensorPosition(Constants.PID_INDEX);
				if(encoderCounts < 0){
					encoderCounts = 0;
				}
			}
			control = getPositionFromEncoderCounts(encoderCounts);
			if(mode != LiftMode.POSITION){
				System.err.println("mode is " + mode + " when control was null!");
			}
			mode = LiftMode.POSITION;
			overrideSpeedSafety = false;
		}
		
		if(mode != null) {
			if (mode == LiftMode.SPEED) {
				if (encoderCounts == null) {
					encoderCounts = master.getSelectedSensorPosition(Constants.PID_INDEX);
				}
				final double position = getPositionFromEncoderCounts(encoderCounts);
				final double desiredSpeed = control;
				final double speed;
				if(overrideSpeedSafety){
					speed = desiredSpeed;
				} else if(desiredSpeed > 0){ // up
					if(position > HIGH_POSITION_SCALE_START){
						speed = desiredSpeed * (1 - position) / (1 - HIGH_POSITION_SCALE_START);
					} else {
						speed = desiredSpeed;
					}
				} else if(desiredSpeed < 0){ // down
					if(position < LOW_POSITION_SCALE_START){
						speed = desiredSpeed * position / LOW_POSITION_SCALE_START;
					} else {
						speed = desiredSpeed;
					}
				} else{
					speed = desiredSpeed;
				}
				master.set(ControlMode.PercentOutput, speed);
				resetControlCache(); // motor safety
				desiredPositionReached = false;
			} else if (mode == LiftMode.POSITION) {// no motor safety
				if (control == 0 && (encoderCounts != null && encoderCounts <= 0)) {
					master.set(ControlMode.Disabled, 0);
					desiredPositionReached = true;
				} else {
					final int position = getEncoderCountsFromPosition(control);
					master.set(ControlMode.Position, position);
					final double currentPosition = min(1, max(0, getPositionFromEncoderCounts(master.getSelectedSensorPosition(Constants.PID_INDEX))));
					desiredPositionReached = abs(currentPosition - control) < DESIRED_REACHED_POSITION_DEADZONE;
				}
			} else {
				throw new UnsupportedOperationException("Unsupported mode: " + mode);
			}
		} else {
			master.set(ControlMode.Disabled, 0);
			resetControlCache();
			desiredPositionReached = false;
		}
	}
	private void resetControlCache(){
		mode = null;
		control = 0.0;
		overrideSpeedSafety = false;
	}
	
}
