package com.first1444.frc.robot2019.vision;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

final class VisionTest {
	
	@Test
	void visionPacketTest(){
		{
			final VisionPacket packet = new ImmutableVisionPacket(-15, 0, -15, 0, 0, 0, 0, 0);
			assertEquals(-15, packet.getRobotX());
			assertEquals(-15, packet.getRobotZ());
			assertEquals(0, packet.getVisionYaw());
			assertEquals(45, packet.getGroundAngle(), .01);
		}
		{
			final VisionPacket packet = new ImmutableVisionPacket(23.75, 0, -62.90, -22.03, 0, 0, 0, 0);
			assertEquals(90, packet.getGroundAngle(), 5);
		}
		
		{
			final VisionPacket packet = new ImmutableVisionPacket(0, 0, -15, 0, 0, 0, 0, 0);
			assertEquals(90, packet.getGroundAngle(), .01);
		}
		
	}
	
}
