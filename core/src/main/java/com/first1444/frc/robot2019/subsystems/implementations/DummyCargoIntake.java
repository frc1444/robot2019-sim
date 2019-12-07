package com.first1444.frc.robot2019.subsystems.implementations;

import com.first1444.frc.robot2019.Constants;
import com.first1444.frc.robot2019.subsystems.CargoIntake;
import com.first1444.frc.util.reportmap.ReportMap;
import me.retrodaredevil.action.SimpleAction;

public class DummyCargoIntake extends SimpleAction implements CargoIntake {
    private static final String INTAKE_SPEED = "Cargo Intake Speed";
    private static final String PIVOT = "Cargo Intake Pivot";
    private final ReportMap reportMap;


    private double intakeSpeed = 0;

    public DummyCargoIntake(ReportMap reportMap) {
        super(true);
        this.reportMap = reportMap;
    }

    @Override
    protected void onUpdate() {
        super.onUpdate();
        reportMap.report(INTAKE_SPEED, Constants.DECIMAL_FORMAT.format(intakeSpeed));
        intakeSpeed = 0;
    }

    @Override
    protected void onStart() {
        super.onStart();
        reportMap.report(INTAKE_SPEED, "enabled");
        reportMap.report(PIVOT, "enabled");
    }

    @Override
    protected void onEnd(boolean peacefullyEnded) {
        super.onEnd(peacefullyEnded);
        reportMap.report(INTAKE_SPEED, "disabled");
        reportMap.report(PIVOT, "disabled");
    }

    @Override
    public void setSpeed(double speed) {
        intakeSpeed = speed;
    }

    @Override
    public void stow() {
        reportMap.report(PIVOT, "stow");
    }

    @Override
    public void pickup() {
        reportMap.report(PIVOT, "pickup");
    }
}
