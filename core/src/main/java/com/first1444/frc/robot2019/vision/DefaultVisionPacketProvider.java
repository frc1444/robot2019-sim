package com.first1444.frc.robot2019.vision;

import com.first1444.frc.robot2019.Perspective;

public class DefaultVisionPacketProvider implements VisionPacketProvider{
	private final Perspective perspective;
	private final VisionSupplier visionSupplier;
	private final int cameraID;
	private final PreferredTargetSelector preferredTargetSelector;
	
	private final long packetValidityTimeMillis;
	
	public DefaultVisionPacketProvider(Perspective perspective, VisionSupplier visionSupplier, int cameraID, PreferredTargetSelector preferredTargetSelector, long packetValidityTimeMillis) {
		this.perspective = perspective;
		this.visionSupplier = visionSupplier;
		this.cameraID = cameraID;
		this.preferredTargetSelector = preferredTargetSelector;
		this.packetValidityTimeMillis = packetValidityTimeMillis;
	}
	
	@Override
	public Perspective getPerspective() {
		return perspective;
	}
	
	@Override
	public VisionPacket getPacket() {
		final VisionInstant visionInstant = visionSupplier.getInstant(cameraID);
		if(visionInstant == null){
			return null;
		}
		if(visionInstant.getTimeMillis() + packetValidityTimeMillis < System.currentTimeMillis()){
			return null;
		}
		final var packets = visionInstant.getVisiblePackets();
		if(packets.isEmpty()){
			return null;
		}
		return preferredTargetSelector.getPreferredTarget(packets);
	}
}
