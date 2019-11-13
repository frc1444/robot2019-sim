package com.first1444.frc.robot2019.vision;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.zeromq.ZContext;
import org.zeromq.ZMQ;

import java.util.*;

import static java.lang.Math.max;
import static java.lang.Math.min;

public class PacketListener extends Thread implements VisionSupplier{
	private static final double MILLIMETERS_IN_INCH = 25.4;
	private final int port;
	private Map<Integer, VisionInstant> visionMap = null;
	public PacketListener(int port){
		this.port = port;
		setDaemon(true);
	}
	@Override
	public VisionInstant getInstant(int cameraID){
		final Map<Integer, VisionInstant> visionMap;
		synchronized (this){
			visionMap = this.visionMap;
		}
		if(visionMap == null){
			return null;
		}
		return visionMap.get(cameraID);
	}
	
	@Override
	public void run() {
		try(ZContext context = new ZContext()) {
			ZMQ.Socket socket = context.createSocket(ZMQ.SUB);
			socket.connect("tcp://10.14.44.5:" + port);
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
			updatePackets(VisionConstants.GSON.fromJson(jsonString, JsonArray.class));
		} catch(IllegalStateException ex){
			ex.printStackTrace();
			System.err.println("Got error while parsing vision!");
		} catch(Exception ex){
			ex.printStackTrace();
			System.err.println("Got exception while parsing vision");
		}
	}
	private void updatePackets(JsonArray jsonArray){
		final Map<Integer, VisionInstant> instantMap = new HashMap<>();
		for(JsonElement instantElement : jsonArray.getAsJsonArray()){
			final JsonObject instantObject = instantElement.getAsJsonObject();
			
			final JsonArray packetArray = instantObject.get("packets").getAsJsonArray();
			final Collection<VisionPacket> packets = new ArrayList<>();
			for (final JsonElement packetElement : packetArray) {
				final JsonObject packetObject = packetElement.getAsJsonObject();
				final VisionPacket packet = new ImmutableVisionPacket(
						packetObject.get("x").getAsDouble() / MILLIMETERS_IN_INCH,
						packetObject.get("y").getAsDouble() / MILLIMETERS_IN_INCH,
						packetObject.get("z").getAsDouble() / MILLIMETERS_IN_INCH,
						packetObject.get("yaw").getAsDouble(),
						packetObject.get("pitch").getAsDouble(),
						packetObject.get("roll").getAsDouble(),
						max(-1, min(1, packetObject.get("imageX").getAsDouble())),
						max(-1, min( 1, packetObject.get("imageY").getAsDouble()))
				);
				packets.add(packet);
			}
			final int cameraID = instantObject.get("cameraId").getAsInt();
			final VisionInstant instant = new ImmutableVisionInstant(packets, System.currentTimeMillis(), cameraID);
			instantMap.put(cameraID, instant);
//			System.out.println(instant);
		}
		synchronized (this) {
			this.visionMap = instantMap;
		}
	}
}
