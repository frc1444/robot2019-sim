package com.first1444.frc.robot2019.event;

public final class SoundEvents {
	private SoundEvents(){ throw new UnsupportedOperationException(); }
	
	public static final String DISABLE = "disable";
	public static final String AUTONOMOUS_ENABLE = "auto.enable";
	public static final String TELEOP_ENABLE = "teleop.enable";
	
	public static final String MATCH_END = "match.end";
	
	public static final String TARGET_FOUND = "vision.found";
	public static final String TARGET_FAILED = "vision.failed";
}
