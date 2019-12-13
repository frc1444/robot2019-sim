package com.first1444.frc.robot2019;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.first1444.frc.robot2019.subsystems.CargoIntake;
import me.retrodaredevil.action.SimpleAction;

public class MotorCargoIntake extends SimpleAction implements CargoIntake {
    /** A negative number in range [-1..0)*/
    private static final double STOW_SPEED = -1.0;
    /** A position number in range (0..1]*/
    private static final double PICKUP_SPEED = 1.0;
    private enum Preset {STOW, PICKUP, NEUTRAL}

    private final BaseMotorController intake;
    private final TalonSRX pivot;

    private double intakeSpeed;
    private Preset preset = Preset.STOW;

    public MotorCargoIntake(BaseMotorController intake, TalonSRX pivot) {
        super(true);
        this.intake = intake;
        this.pivot = pivot;

        intake.configFactoryDefault(Constants.INIT_TIMEOUT);
        pivot.configFactoryDefault(Constants.INIT_TIMEOUT);

        intake.setInverted(InvertType.InvertMotorOutput);
        intake.setNeutralMode(NeutralMode.Brake);


        pivot.setNeutralMode(NeutralMode.Brake);
        pivot.setInverted(InvertType.InvertMotorOutput);

        pivot.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, Constants.INIT_TIMEOUT); // normally closed - important
        pivot.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, Constants.INIT_TIMEOUT); // normally closed - important
    }

    @Override
    protected void onUpdate() {
        super.onUpdate();
        intake.set(ControlMode.PercentOutput, intakeSpeed);
        intakeSpeed = 0;

        final boolean forwardLimitSwitch = !pivot.getSensorCollection().isFwdLimitSwitchClosed(); // normally closed
        switch (preset){
            case STOW:
                pivot.set(ControlMode.PercentOutput, STOW_SPEED); // rely on the limit switch
                break;
            case PICKUP:
                if(forwardLimitSwitch){
                    pivot.set(ControlMode.Disabled, 0);
                    preset = Preset.NEUTRAL;
                } else {
                    pivot.set(ControlMode.PercentOutput, PICKUP_SPEED);
                }
                break;
            case NEUTRAL:
                pivot.set(ControlMode.Disabled, 0);
                break;
            default:
                throw new UnsupportedOperationException();
        }
    }

    @Override
    public void setSpeed(double speed) {
        intakeSpeed = speed;
    }

    @Override
    public void stow() {
        preset = Preset.STOW;
        System.out.println("stowing");
    }

    @Override
    public void pickup() {
        preset = Preset.PICKUP;
        System.out.println("picking up");
    }
}
