package com.first1444.frc.robot2019.event;

/**
 * @deprecated Because this was used for sounds, it is now deprecated in favor of a SoundCreator
 */
@Deprecated
public interface EventSender {
    void sendEvent(String data);
    void sendEvents(String... data);
}
