package com.first1444.frc.robot2019.vision;

import java.util.Collection;
import java.util.Collections;

class ImmutableVisionInstant implements VisionInstant {
	private final Collection<VisionPacket> packets;
	private final long timeMillis;
	private final int cameraID;
	public ImmutableVisionInstant(Collection<VisionPacket> packets, long timeMillis, int cameraID) {
		this.packets = Collections.unmodifiableCollection(packets);
		this.timeMillis = timeMillis;
		this.cameraID = cameraID;
	}
	
	@Override
	public Collection<VisionPacket> getVisiblePackets() {
		return packets;
	}
	
	@Override
	public long getTimeMillis() {
		return timeMillis;
	}
	
	@Override
	public int getCameraID() {
		return cameraID;
	}
	
	@Override
	public String toString() {
		return "ImmutableVisionInstant{" +
				"packets=" + packets +
				", timeMillis=" + timeMillis +
				", cameraID=" + cameraID +
				'}';
	}
}
