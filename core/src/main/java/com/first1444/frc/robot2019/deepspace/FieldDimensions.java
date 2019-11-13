package com.first1444.frc.robot2019.deepspace;

import static com.first1444.sim.api.MeasureUtil.inchesToMeters;

public final class FieldDimensions {
	private FieldDimensions() { throw new UnsupportedOperationException(); }
	
	public static final double HAB_FLAT_DISTANCE = inchesToMeters(35.5); // estimate
	public static final double HAB_LIP_DISTANCE = inchesToMeters(48.28) - HAB_FLAT_DISTANCE; // estimate
}
