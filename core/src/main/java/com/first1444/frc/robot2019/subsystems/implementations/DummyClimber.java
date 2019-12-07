package com.first1444.frc.robot2019.subsystems.implementations;

import com.first1444.frc.robot2019.Constants;
import com.first1444.frc.robot2019.subsystems.Climber;
import com.first1444.frc.util.reportmap.ReportMap;
import me.retrodaredevil.action.SimpleAction;

public class DummyClimber extends SimpleAction implements Climber {
    private static final String CLIMB_SPEED = "Climb Climb Speed";
    private static final String DRIVE_SPEED = "Climb Drive Speed";
    private final ReportMap reportMap;

    public DummyClimber(ReportMap reportMap) {
        super(true);
        this.reportMap = reportMap;
    }

    @Override
    protected void onStart() {
        super.onStart();
        reportMap.report(CLIMB_SPEED, "robot enabled");
        reportMap.report(DRIVE_SPEED, "robot enabled");
    }

    @Override
    protected void onEnd(boolean peacefullyEnded) {
        super.onEnd(peacefullyEnded);
        reportMap.report(CLIMB_SPEED, "robot disabled");
        reportMap.report(DRIVE_SPEED, "robot disabled");
    }

    @Override
    public void setClimbSpeed(double speed) {
        reportMap.report(CLIMB_SPEED, Constants.DECIMAL_FORMAT.format(speed));
    }

    @Override
    public void setDriveSpeed(double speed) {
        reportMap.report(DRIVE_SPEED, Constants.DECIMAL_FORMAT.format(speed));
    }
}
