package com.first1444.frc.robot2019.vision;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import static java.lang.Math.abs;
import static java.lang.Math.hypot;

@Deprecated
public class BestVisionPacketSelector implements PreferredTargetSelector {
	@Override
	public VisionPacket getPreferredTarget(Collection<? extends VisionPacket> visiblePackets) {
		if(visiblePackets.isEmpty())
			throw new IllegalArgumentException();
		
		visiblePackets = goodFilter(visiblePackets);
		if(visiblePackets.isEmpty()){
			return null;
		}
		
		List<? extends VisionPacket> packets = filterAngles(visiblePackets);
		if(packets.size() == 1){
			return packets.get(0);
		}
		if(packets.isEmpty()){
			return filterDistance(visiblePackets);
		}
		return filterDistance(packets);
	}
	private List<? extends VisionPacket> goodFilter(Collection<? extends VisionPacket> visionPackets){
		final List<VisionPacket> packets = new ArrayList<>(visionPackets);
		packets.removeIf(packet -> abs(packet.getVisionYaw()) > 50);
		return packets;
	}
	private List<? extends VisionPacket> filterAngles(Collection<? extends VisionPacket> visionPackets){
		final List<VisionPacket> packets = new ArrayList<>(visionPackets);
		packets.removeIf(packet -> abs(packet.getVisionYaw()) > 45 || abs(packet.getVisionPitch()) > 90 || abs(packet.getVisionRoll()) > 45);
		if(packets.isEmpty()){
			return new ArrayList<>(visionPackets);
		}
		return packets;
	}
	private VisionPacket filterDistance(Collection<? extends VisionPacket> visionPackets){
		VisionPacket closest = null;
		double closestDistance = 0;
		for(VisionPacket packet : visionPackets){
			final double distance = hypot(packet.getRobotX() * 3, packet.getRobotX()); // multiply by 3 to make targets off on the x axis by a lot be filtered out
			if(closest == null || closestDistance > distance){
				closest = packet;
				closestDistance = distance;
			}
		}
		return closest;
	}
	
}
