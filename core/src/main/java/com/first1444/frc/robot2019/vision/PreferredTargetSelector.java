package com.first1444.frc.robot2019.vision;

import java.util.Collection;

@Deprecated
public interface PreferredTargetSelector {
    /**
     *
     * @param visiblePackets The visible vision packets. This cannot be empty
     * @return The preferred vision target. Possibly null if none of the packets were good
     * @throws IllegalArgumentException If visiblePackets is empty. Optional.
     */
    VisionPacket getPreferredTarget(Collection<? extends VisionPacket> visiblePackets);
}
