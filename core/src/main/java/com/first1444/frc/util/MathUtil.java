package com.first1444.frc.util;

/**
 * Most code stolen from https://github.com/retrodaredevil/track-shooter/blob/master/core/src/me/retrodaredevil/game/trackshooter/util/MathUtil.java
 */
public final class MathUtil {
	private MathUtil(){ throw new UnsupportedOperationException(); }
	/**
	 *
	 * @param wpilibDegrees The degrees to remap to the correct way
	 * @return
	 */
	public static double toEulerDegrees(double wpilibDegrees){
		return 90 - wpilibDegrees; // == euler degrees
	}
	public static double toWPIDegrees(double eulerDegrees){
		return 90 - eulerDegrees;
	}
	
}
