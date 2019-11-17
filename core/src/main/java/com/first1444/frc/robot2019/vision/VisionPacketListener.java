package com.first1444.frc.robot2019.vision;

import com.first1444.sim.api.Clock;
import com.first1444.sim.api.Transform;
import com.first1444.sim.api.surroundings.Surrounding;
import com.first1444.sim.api.surroundings.Surrounding3DExtra;
import com.first1444.sim.api.surroundings.SurroundingProvider;
import com.google.gson.*;
import org.jetbrains.annotations.NotNull;
import org.zeromq.ZContext;
import org.zeromq.ZMQ;

import java.util.*;

public class VisionPacketListener extends Thread implements SurroundingProvider {
	public static final Gson GSON = new GsonBuilder()
			.create();

	private final Clock clock;
	private final Map<Integer, Double> cameraOffsetDegreesMap;
	private final String ip;
	private final int port;
	private List<Surrounding> surroundingList = null;
	public VisionPacketListener(Clock clock, Map<Integer, Double> cameraOffsetDegreesMap, String ip, int port){
		this.clock = clock;
		this.cameraOffsetDegreesMap = cameraOffsetDegreesMap;
		this.ip = ip;
		this.port = port;
		setDaemon(true);
	}
	@NotNull
	@Override
	public List<Surrounding> getSurroundings() {
		final List<Surrounding> r;
		synchronized (this) {
			r = this.surroundingList;
		}
		if(r == null){
			return Collections.emptyList();
		}
		return r;
	}

	@Override
	public void run() {
		try(ZContext context = new ZContext()) {
			ZMQ.Socket socket = context.createSocket(ZMQ.SUB);
			socket.connect("tcp://" + ip + ":" + port);
			socket.setLinger(0);
			socket.subscribe("".getBytes());
			
			while (!Thread.currentThread().isInterrupted()) {
				final String reply = socket.recvStr(0);
				if(reply != null) {
					updatePackets(reply);
				}
			}
		}
	}
	
	private void updatePackets(String jsonString){
		try {
			updatePackets(clock.getTimeSeconds(), GSON.fromJson(jsonString, JsonArray.class));
		} catch(IllegalStateException ex){
			ex.printStackTrace();
			System.err.println("Got error while parsing vision!");
		} catch(Exception ex){
			ex.printStackTrace();
			System.err.println("Got exception while parsing vision");
		}
	}
	private void updatePackets(double timestamp, JsonArray jsonArray){
		// TODO This code was refactored from last years code to try to be compatible with the protocol used in 2019, we should change the protocol for 2020
        final List<Surrounding> surroundings = new ArrayList<>(jsonArray.size());
		for(JsonElement instantElement : jsonArray){
			final JsonObject instantObject = instantElement.getAsJsonObject();
			
			final JsonArray packetArray = instantObject.get("packets").getAsJsonArray();
			final int cameraID = instantObject.get("cameraId").getAsInt();
			final double offsetDegrees = cameraOffsetDegreesMap.get(cameraID);
			for (final JsonElement packetElement : packetArray) {
				final JsonObject packetObject = packetElement.getAsJsonObject();
				final Transform correctTransform = Transform.transformRadians(
						packetObject.get("x").getAsDouble() / 1000,
						packetObject.get("z").getAsDouble() / 1000,
						0.0
				).getReversed();
				final Surrounding surrounding = new Surrounding(
						Transform.transformDegrees(
								correctTransform.getX(),
								correctTransform.getY(),
								packetObject.get("yaw").getAsDouble()
						).rotateDegrees(offsetDegrees),
						timestamp,
						Surrounding3DExtra.fromDegrees(
								packetObject.get("y").getAsDouble() / 1000,
								packetObject.get("pitch").getAsDouble(),
								packetObject.get("roll").getAsDouble()
						)
				);
				surroundings.add(surrounding);
			}
		}
		synchronized (this) {
		    this.surroundingList = surroundings;
		}
	}

}
