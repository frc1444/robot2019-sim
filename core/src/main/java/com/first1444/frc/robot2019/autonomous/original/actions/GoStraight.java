package com.first1444.frc.robot2019.autonomous.original.actions;

import com.first1444.frc.robot2019.subsystems.swerve.SwerveDistanceTracker;
import com.first1444.sim.api.MathUtil;
import com.first1444.sim.api.Rotation2;
import com.first1444.sim.api.Vector2;
import com.first1444.sim.api.drivetrain.swerve.SwerveDrive;
import com.first1444.sim.api.sensors.Orientation;
import me.retrodaredevil.action.SimpleAction;
import org.jetbrains.annotations.Nullable;

import static java.lang.Math.*;

@Deprecated
public class GoStraight extends SimpleAction {
    private final double distanceMeters;
    private final Vector2 translate;
    @Nullable
    private final Rotation2 faceDirection;
    private final SwerveDrive drive;
    private final Orientation orientation;

    private SwerveDistanceTracker tracker = null;


    private GoStraight(
            double distanceMeters, Vector2 translate, @Nullable Rotation2 faceDirection,
            SwerveDrive drive, Orientation orientation
    ) {
        super(true);
        this.distanceMeters = distanceMeters;
        this.translate = translate;
        this.faceDirection = faceDirection;
        this.drive = drive;
        this.orientation = orientation;
    }
    public static GoStraight createGoStraightAtHeading(
            double distanceMeters, double speed, Rotation2 heading, Rotation2 faceDirection,
            SwerveDrive drive, Orientation orientation
    ){
        return new GoStraight(distanceMeters, new Vector2(heading.getCos() * speed, heading.getSin() * speed), faceDirection, drive, orientation);
    }

    @Override
    protected void onStart() {
        super.onStart();
        tracker = new SwerveDistanceTracker(drive);
    }

    @Override
    protected void onUpdate() {
        super.onUpdate();

        final SwerveDrive drive = tracker.getDrive();
        final double minChange;
        final Rotation2 currentOrientation = orientation.getOrientation();
        if(faceDirection != null) {
            minChange = MathUtil.minChange(faceDirection.getDegrees(), currentOrientation.getDegrees(), 360);
        } else {
            minChange = 0;
        }
        final double turnAmount = .75 * max(-1, min(1, minChange / -40));
        drive.setControl(translate.rotate(currentOrientation.unaryMinus()), turnAmount, 1);

        setDone(tracker.calculateDistance() >= distanceMeters);

    }
}
