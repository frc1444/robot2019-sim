package com.first1444.frc.robot2019.event;

public interface EventSender {
	void sendEvent(String data);
	void sendEvents(String... data);
}
