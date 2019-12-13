package com.first1444.frc.robot2019.autonomous.original;

import com.first1444.frc.robot2019.autonomous.creator.OperatorActionCreator;
import com.first1444.frc.robot2019.autonomous.creator.TestOperatorActionCreator;
import com.first1444.frc.robot2019.autonomous.creator.TestVisionPlacementCreator;
import com.first1444.frc.robot2019.autonomous.creator.VisionPlacementCreator;
import com.first1444.frc.util.autonomous.creator.FrcLogActionCreator;
import com.first1444.frc.util.autonomous.creator.LogActionCreator;
import com.first1444.frc.util.autonomous.creator.OriginalSwerveDriveActionCreator;
import com.first1444.frc.util.autonomous.creator.TestOriginalSwerveDriveActionCreator;
import com.first1444.sim.api.frc.sim.PrintStreamFrcLogger;

import java.io.PrintStream;

public class TestOriginalAutonActionCreator implements OriginalAutonActionCreator {
    private final LogActionCreator logActionCreator;
    private final OriginalSwerveDriveActionCreator driveActionCreator;
    private final VisionPlacementCreator visionPlacementCreator;
    private final OperatorActionCreator operatorActionCreator;


    public TestOriginalAutonActionCreator(PrintStream out) {
        logActionCreator = new FrcLogActionCreator(out, new PrintStreamFrcLogger(out, out));
        driveActionCreator = new TestOriginalSwerveDriveActionCreator(logActionCreator);
        visionPlacementCreator = new TestVisionPlacementCreator(logActionCreator);
        operatorActionCreator = new TestOperatorActionCreator(logActionCreator);
    }
    public TestOriginalAutonActionCreator(){
        this(System.out);
    }

    @Override
    public LogActionCreator getLogCreator() {
        return logActionCreator;
    }

    @Override
    public OriginalSwerveDriveActionCreator getDriveCreator() {
        return driveActionCreator;
    }

    @Override
    public VisionPlacementCreator getVisionCreator() {
        return visionPlacementCreator;
    }

    @Override
    public OperatorActionCreator getOperatorCreator() {
        return operatorActionCreator;
    }
}
