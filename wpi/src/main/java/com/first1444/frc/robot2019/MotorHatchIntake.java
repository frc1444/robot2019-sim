package com.first1444.frc.robot2019;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.first1444.frc.robot2019.subsystems.HatchIntake;
import me.retrodaredevil.action.SimpleAction;

public class MotorHatchIntake extends SimpleAction implements HatchIntake {
	private static final double GRAB_SPEED = .9;
	
	private static final double PIVOT_SPEED_BACK = -1.0;
	private static final double PIVOT_SPEED_DOWN = 1;
	private static final double PIVOT_DOWN_STALL = .6;
	
	private static final double STOW_SPEED_BACK = -1.0;
//	private static final double STOW_SPEED_OUT = 1.0;
	
	private static final int STOW_MOTOR_MAX_ENCODER_COUNTS = 10000;
	private static final int STOW_MOTOR_IS_OUT_ENCODER_COUNTS = 9000;
	private static final int STOW_MOTOR_IS_OUT_ENCODER_COUNTS_DEADZONE = 8500;
//	private static final int STOW_MOTOR_MAX_ENCODER_COUNTS = 11082;
	private enum GrabMode {NEUTRAL, GRAB, DROP}
	private enum Preset {GROUND, NORMAL, STOWED, NEUTRAL}
	
	/** The grab motor. This uses two limit switches. One for reverse and one for forward*/
	private final TalonSRX grabMotor;
	/** The stow motor. This has a reverse limit switch and uses an encoder*/
	private final TalonSRX stowMotor;
	/** The pivot motor. This uses two limit switches. One for reverse and one for forward. This is not enforced.*/
	private final TalonSRX pivotMotor;
	
	private GrabMode grabMode = GrabMode.NEUTRAL;
	private Preset preset = Preset.NEUTRAL;
	
	private boolean isStowOut = false;
	
	private boolean desiredPositionReached = false;
	
	public MotorHatchIntake(TalonSRX grabMotor, TalonSRX stowMotor, TalonSRX pivotMotor) {
		super(true);
		this.grabMotor = grabMotor;
		this.pivotMotor = pivotMotor;
		this.stowMotor = stowMotor;
		
		grabMotor.configFactoryDefault(Constants.INIT_TIMEOUT);
		stowMotor.configFactoryDefault(Constants.INIT_TIMEOUT);
		pivotMotor.configFactoryDefault(Constants.INIT_TIMEOUT);
		// Grab
		grabMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, Constants.INIT_TIMEOUT);
		grabMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, Constants.INIT_TIMEOUT);
		grabMotor.setNeutralMode(NeutralMode.Brake);
		
		// Stow
//		stowMotor.configForwardSoftLimitEnable(true, Constants.INIT_TIMEOUT);
//		stowMotor.configForwardSoftLimitThreshold(STOW_MOTOR_MAX_ENCODER_COUNTS, Constants.INIT_TIMEOUT);
		stowMotor.setNeutralMode(NeutralMode.Brake);
		stowMotor.config_kP(Constants.SLOT_INDEX, .3, Constants.INIT_TIMEOUT); // TODO CTRE PID
		stowMotor.setInverted(InvertType.InvertMotorOutput); // motor is inverted but encoder is not
		stowMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, Constants.INIT_TIMEOUT);
		stowMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, Constants.INIT_TIMEOUT); // sometimes this gets tripped, so just disabled it
		
		// Pivot // I plugged this in correctly so we shouldn't have to set inverted
		pivotMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, Constants.INIT_TIMEOUT);
		pivotMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, Constants.INIT_TIMEOUT);
		pivotMotor.setNeutralMode(NeutralMode.Brake);
		pivotMotor.overrideLimitSwitchesEnable(false);
		
	}
	private boolean isStowFullyForward(){
		return stowMotor.getSelectedSensorPosition(Constants.PID_INDEX) >= STOW_MOTOR_IS_OUT_ENCODER_COUNTS;
	}
	private boolean isPivotBack(){
		return !pivotMotor.getSensorCollection().isRevLimitSwitchClosed(); // normally closed
	}
	
	@Override
	protected void onUpdate() {
		super.onUpdate();
		switch(grabMode){
			case NEUTRAL:
				grabMotor.set(ControlMode.Disabled, 0);
				break;
			case GRAB:
				grabMotor.set(ControlMode.PercentOutput, GRAB_SPEED);
				break;
			case DROP:
				grabMotor.set(ControlMode.PercentOutput, -GRAB_SPEED);
				break;
		}
		final boolean stowReverseLimit = !stowMotor.getSensorCollection().isRevLimitSwitchClosed(); // normally closed
//		if(stowReverseLimit){
//			stowMotor.setSelectedSensorPosition(0);
//		}
		final boolean stowForward = isStowFullyForward();
		final boolean stowPastDeadzone = stowMotor.getSelectedSensorPosition(Constants.PID_INDEX) >= STOW_MOTOR_IS_OUT_ENCODER_COUNTS_DEADZONE;
		final boolean pivotBack = !pivotMotor.getSensorCollection().isRevLimitSwitchClosed(); // normally closed
		final boolean pivotDown = !pivotMotor.getSensorCollection().isFwdLimitSwitchClosed(); // normally closed
//		if(pivotDown){ // if we start with it down or if it's down, assume all the way out
//			stowMotor.setSelectedSensorPosition(STOW_MOTOR_MAX_ENCODER_COUNTS);
//		}
		if(stowForward){
			isStowOut = true;
		} else if(!stowPastDeadzone){
			isStowOut = false;
		}
		switch(preset){
			case GROUND:
				if(stowForward){
					stowMotor.set(ControlMode.Disabled, 0);
				} else {
					stowMotor.set(ControlMode.Position, STOW_MOTOR_MAX_ENCODER_COUNTS);
				}
				if(isStowFullyForward()){ // only bring down pivot if fully forward
					if(pivotDown) {
						pivotMotor.set(ControlMode.PercentOutput, PIVOT_DOWN_STALL);
					} else {
						pivotMotor.set(ControlMode.PercentOutput, PIVOT_SPEED_DOWN);
					}
					desiredPositionReached = pivotDown; // we want the pivot to be down
				} else { // keep pivot back until ready
					if(pivotBack) {
						pivotMotor.set(ControlMode.Disabled, 0);
					} else {
						pivotMotor.set(ControlMode.PercentOutput, PIVOT_SPEED_BACK);
					}
					desiredPositionReached = false;
				}
				break;
			case NORMAL:
				if(stowForward || (stowPastDeadzone && isStowOut)){
					stowMotor.set(ControlMode.Disabled, 0);
				} else {
					stowMotor.set(ControlMode.Position, STOW_MOTOR_MAX_ENCODER_COUNTS);
				}
				if(pivotBack){
					pivotMotor.set(ControlMode.Disabled, 0);
				} else {
					pivotMotor.set(ControlMode.PercentOutput, PIVOT_SPEED_BACK);
				}
				desiredPositionReached = stowForward && isPivotBack();
				break;
			case STOWED:
				if(pivotBack){
					pivotMotor.set(ControlMode.Disabled, 0);
				} else {
					pivotMotor.set(ControlMode.PercentOutput, PIVOT_SPEED_BACK);
				}
				if(isPivotBack()){
					stowMotor.set(ControlMode.PercentOutput, STOW_SPEED_BACK);
					desiredPositionReached = stowReverseLimit;
				} else {
					stowMotor.set(ControlMode.Disabled, 0);
					desiredPositionReached = false;
				}
				break;
			case NEUTRAL:
//				System.out.println("In neutral mode!!! The hatch intake might fall");
				desiredPositionReached = false;
				break;
			default:
				throw new UnsupportedOperationException("unknown preset: " + preset);
		}
	}
	
	@Override
	protected void onEnd(boolean peacefullyEnded) {
		super.onEnd(peacefullyEnded);
		grabMode = GrabMode.NEUTRAL;
		preset = Preset.NEUTRAL;
		
		grabMotor.set(ControlMode.Disabled, 0);
		pivotMotor.set(ControlMode.Disabled, 0);
		stowMotor.set(ControlMode.Disabled, 0);
	}
	
	@Override
	public void hold(){
		grabMode = GrabMode.GRAB;
	}
	@Override
	public void drop(){
		grabMode = GrabMode.DROP;
	}
	
	@Override
	public void neutralHold() {
		grabMode = GrabMode.NEUTRAL;
	}
	
	@Override
	public void groundPosition(){
		preset = Preset.GROUND;
	}
	@Override
	public void readyPosition(){
		preset = Preset.NORMAL;
	}
	@Override
	public void stowedPosition(){
		preset = Preset.STOWED;
	}
	
	@Override
	public boolean isDesiredPositionReached() {
		return desiredPositionReached;
	}
	
}
