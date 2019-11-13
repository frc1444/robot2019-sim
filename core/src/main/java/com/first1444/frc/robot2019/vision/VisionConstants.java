package com.first1444.frc.robot2019.vision;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

public final class VisionConstants {
	private VisionConstants(){ throw new UnsupportedOperationException(); }
	
	public static final Gson GSON = new GsonBuilder()
			.create();
}
