package com.first1444.frc.robot2019.vision;

import com.first1444.frc.robot2019.Perspective;

public interface VisionPacketProvider {
	Perspective getPerspective();
	VisionPacket getPacket();
}
