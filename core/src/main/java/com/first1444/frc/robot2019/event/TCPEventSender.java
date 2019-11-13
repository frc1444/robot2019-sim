package com.first1444.frc.robot2019.event;

import org.zeromq.ZContext;
import org.zeromq.ZMQ;

public class TCPEventSender implements EventSender, AutoCloseable {
	
	private final ZContext context;
	private final ZMQ.Socket socket;
	
	public TCPEventSender(int port) {
		context = new ZContext();
		socket = context.createSocket(ZMQ.PUB);
		socket.bind("tcp://*:" + port);
	}
	
	
	@Override
	public void sendEvent(String data) {
		socket.send(data.replaceAll("\n", "\\n") + "\n");
	}
	
	@Override
	public void sendEvents(String... data) {
		final StringBuilder builder = new StringBuilder();
		for(String message : data){
			builder.append(message.replaceAll("\n", "\\n"));
			builder.append('\n');
		}
		socket.send(builder.toString());
	}
	
	@Override
	public void close() {
		socket.close();
		context.close();
	}
}
