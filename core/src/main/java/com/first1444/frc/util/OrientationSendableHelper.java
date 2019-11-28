package com.first1444.frc.util;

import com.first1444.dashboard.shuffleboard.ComponentMetadataHelper;
import com.first1444.dashboard.shuffleboard.SendableComponent;
import com.first1444.dashboard.shuffleboard.ShuffleboardContainer;
import com.first1444.dashboard.shuffleboard.implementations.GyroMetadataHelper;
import com.first1444.sim.api.sensors.Orientation;
import com.first1444.sim.api.sensors.OrientationSendable;

public class OrientationSendableHelper {

	public static void addOrientation(ShuffleboardContainer container, Orientation orientation){
		container.add("Orientation",
				new SendableComponent<>(new OrientationSendable(orientation)),
				(metadata) -> {
                    new ComponentMetadataHelper(metadata).setSize(2, 3).setPosition(9, 1);
					new GyroMetadataHelper(metadata).setMajorTickSpacing(45.0).setStartingAngle(90).setCounterClockwise(true);
				}
		);
	}
}
