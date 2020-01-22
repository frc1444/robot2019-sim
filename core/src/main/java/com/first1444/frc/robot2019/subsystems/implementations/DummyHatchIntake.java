package com.first1444.frc.robot2019.subsystems.implementations;

import com.first1444.frc.robot2019.subsystems.HatchIntake;
import com.first1444.frc.util.reportmap.ReportMap;
import me.retrodaredevil.action.SimpleAction;

public class DummyHatchIntake extends SimpleAction implements HatchIntake {
    private static final String GRAB = "Hatch Intake Grab State";
    private static final String PIVOT = "Hatch Intake Pivot";
    private final ReportMap reportMap;

    public DummyHatchIntake(ReportMap reportMap) {
        super(true);
        this.reportMap = reportMap;
    }

    @Override
    protected void onStart() {
        super.onStart();
        reportMap.report(GRAB, "enabled");
        reportMap.report(PIVOT, "enabled");
    }

    @Override
    protected void onEnd(boolean peacefullyEnded) {
        super.onEnd(peacefullyEnded);
        reportMap.report(GRAB, "disabled");
        reportMap.report(PIVOT, "disabled");
    }

    @Override
    public void hold() {
        reportMap.report(GRAB, "hold");
    }

    @Override
    public void drop() {
        reportMap.report(GRAB, "drop");
    }

    @Override
    public void neutralHold() {
        reportMap.report(GRAB, "neutral");
    }

    @Override
    public void readyPosition() {
        reportMap.report(PIVOT, "ready position");
    }

    @Override
    public void stowedPosition() {
        reportMap.report(PIVOT, "stowed position");
    }

    @Override
    public boolean isDesiredPositionReached() {
        return true;
    }
}
