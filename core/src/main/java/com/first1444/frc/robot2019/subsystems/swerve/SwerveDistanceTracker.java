package com.first1444.frc.robot2019.subsystems.swerve;

import com.first1444.sim.api.drivetrain.swerve.SwerveDrive;
import com.first1444.sim.api.drivetrain.swerve.SwerveModule;

import java.util.Arrays;
import java.util.List;

/**
 * Calculates the distance traveled since the creation of this object
 */
@Deprecated
public class SwerveDistanceTracker {
    private final SwerveDrive drive;
    private final double[] startingPositions;

    public SwerveDistanceTracker(SwerveDrive drive){
        this.drive = drive;
        startingPositions = getPositions(drive.getDrivetrainData().getModules());
    }

    public static double[] getPositions(List<? extends SwerveModule> modules){
        final double[] positions = new double[modules.size()];
        int i = 0;
        for(SwerveModule module : modules){
            positions[i] = module.getDistanceTraveledMeters();
            i++;
        }
        return positions;
    }
    public SwerveDrive getDrive(){
        return drive;
    }
    public double calculateDistance(){
        double[] currentPositions = getPositions(drive.getDrivetrainData().getModules());
        double[] distanceTraveled = new double[currentPositions.length];
        for(int i = 0; i < currentPositions.length; i++){
            distanceTraveled[i] = currentPositions[i] - startingPositions[i];
        }
        Arrays.sort(distanceTraveled);
        final double middle1 = distanceTraveled[distanceTraveled.length / 2];
        final Double middle2;
        if(distanceTraveled.length % 2 == 0){
            middle2 = distanceTraveled[(distanceTraveled.length + 1) / 2];
        } else {
            middle2 = null;
        }
        return middle2 == null
                ? middle1
                : (middle1 + middle2) / 2.0;
    }
}
