package com.first1444.frc.robot2019;

import java.util.function.BooleanSupplier;

public class DynamicRobotDimensions implements RobotDimensions {
    private final RobotDimensions dimensions;
    private final BooleanSupplier isCameraIDSwitched;

    public DynamicRobotDimensions(RobotDimensions dimensions, BooleanSupplier isCameraIDSwitched) {
        this.dimensions = dimensions;
        this.isCameraIDSwitched = isCameraIDSwitched;
    }

    @Override
    public Perspective getHatchManipulatorPerspective() {
        return dimensions.getHatchManipulatorPerspective();
    }

    @Override
    public Perspective getCargoManipulatorPerspective() {
        return dimensions.getCargoManipulatorPerspective();
    }

    @Override
    public int getHatchCameraID() {
        return isCameraIDSwitched.getAsBoolean() ? dimensions.getCargoCameraID() : dimensions.getHatchCameraID();
    }

    @Override
    public int getCargoCameraID() {
        return isCameraIDSwitched.getAsBoolean() ? dimensions.getHatchCameraID() : dimensions.getCargoCameraID();
    }

    @Override
    public double getHatchManipulatorActiveExtendDistanceMeters() {
        return dimensions.getHatchManipulatorActiveExtendDistanceMeters();
    }

    @Override public double getHatchSideWidthMeters() { return dimensions.getHatchSideWidthMeters(); }
    @Override public double getCargoSideWidthMeters() { return dimensions.getCargoSideWidthMeters(); }
    @Override public double getHatchSideDepthMeters() { return dimensions.getHatchSideDepthMeters(); }
    @Override public double getCargoSideDepthMeters() { return dimensions.getCargoSideDepthMeters(); }
}
