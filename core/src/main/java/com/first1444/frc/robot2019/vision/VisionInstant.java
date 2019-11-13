package com.first1444.frc.robot2019.vision;

import java.util.Collection;

public interface VisionInstant {
	Collection<VisionPacket> getVisiblePackets();
	long getTimeMillis();
	int getCameraID();
}
