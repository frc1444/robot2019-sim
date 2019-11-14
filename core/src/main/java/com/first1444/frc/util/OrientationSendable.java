package com.first1444.frc.util;

import com.first1444.sim.api.sensors.Orientation;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

import java.util.Map;
import java.util.function.Supplier;

import static com.first1444.frc.util.MathUtil.toWPIDegrees;

public class OrientationSendable extends SendableBase {
	private final Supplier<Orientation> orientationSupplier;
	
	private OrientationSendable(Supplier<Orientation> orientationSupplier) {
		this.orientationSupplier = orientationSupplier;
	}
	public static void addOrientation(ShuffleboardContainer container, Supplier<Orientation> orientationSupplier){
		final ShuffleboardLayout layout = container.getLayout("Orientation", BuiltInLayouts.kList).withSize(2, 3).withPosition(9, 1);
		layout.add("Gyro", new OrientationSendable(orientationSupplier)).withProperties(Map.of("Major tick spacing", 360));
		layout.add("Value", new SendableBase() {
			@Override
			public void initSendable(SendableBuilder builder) {
				builder.addDoubleProperty("Value", () -> orientationSupplier.get().getOrientationDegrees(), null);
			}
		});
	}
	
	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("Gyro");
		builder.addDoubleProperty("Value", this::getWPIAngle, null);
	}
	private double getWPIAngle(){
		return toWPIDegrees(orientationSupplier.get().getOrientationDegrees());
	}
}
