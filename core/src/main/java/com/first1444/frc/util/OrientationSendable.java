package com.first1444.frc.util;

import com.first1444.dashboard.ActiveComponent;
import com.first1444.dashboard.BasicDashboard;
import com.first1444.dashboard.advanced.Sendable;
import com.first1444.dashboard.advanced.SendableHelper;
import com.first1444.dashboard.shuffleboard.ComponentMetadataHelper;
import com.first1444.dashboard.shuffleboard.SendableComponent;
import com.first1444.dashboard.shuffleboard.ShuffleboardContainer;
import com.first1444.dashboard.shuffleboard.implementations.ShuffleboardLayoutComponent;
import com.first1444.dashboard.value.BasicValue;
import com.first1444.dashboard.value.ValueProperty;
import com.first1444.dashboard.value.implementations.PropertyActiveComponent;
import com.first1444.sim.api.sensors.Orientation;
import org.jetbrains.annotations.NotNull;

import java.util.Map;

import static com.first1444.frc.util.MathUtil.toWPIDegrees;

public class OrientationSendable implements Sendable<ActiveComponent> {
	private final Orientation orientation;

	public OrientationSendable(Orientation orientation) {
		this.orientation = orientation;
	}

	public static void addOrientation(ShuffleboardContainer container, Orientation orientation){
		final ShuffleboardContainer layout = container.add("Orientation", ShuffleboardLayoutComponent.LIST, (metadata) -> new ComponentMetadataHelper(metadata).setSize(2, 3).setPosition(9, 1));
		layout.add("Gyro",
				new SendableComponent<>(new OrientationSendable(orientation)),
				(metadata) -> new ComponentMetadataHelper(metadata).setProperties(Map.of("Major tick spacing", BasicValue.makeDouble(360)))
		);
//		layout.add("Value", new endableBase() { // TODO implement
//			@Override
//			public void initSendable(endableBuilder builder) {
//				builder.addDoubleProperty("Value", () -> orientationSupplier.get().getOrientationDegrees(), null);
//			}
//		});
	}
	
	private double getWPIAngle(){
		return toWPIDegrees(orientation.getOrientationDegrees());
	}

	@NotNull
	@Override
	public ActiveComponent init(@NotNull String s, @NotNull BasicDashboard basicDashboard) {
	    new SendableHelper(basicDashboard).setDashboardType("Gyro");
		return new PropertyActiveComponent(s, basicDashboard.get("Value"), ValueProperty.createGetOnly(() -> BasicValue.makeDouble(getWPIAngle())));
	}
}
