package com.first1444.frc.robot2019.autonomous.actions.vision;

import com.first1444.frc.robot2019.autonomous.actions.DistanceAwayLinkedAction;
import com.first1444.frc.robot2019.event.EventSender;
import com.first1444.frc.robot2019.event.SoundEvents;
import com.first1444.frc.robot2019.vision.ImmutableVisionPacket;
import com.first1444.frc.robot2019.vision.VisionPacket;
import com.first1444.frc.robot2019.vision.VisionPacketProvider;
import com.first1444.frc.util.MathUtil;
import com.first1444.sim.api.Vector2;
import com.first1444.sim.api.drivetrain.swerve.SwerveDrive;
import com.first1444.sim.api.sensors.Orientation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import me.retrodaredevil.action.Action;
import me.retrodaredevil.action.SimpleAction;

import java.util.Objects;
import java.util.function.Supplier;

import static java.lang.Math.*;

@Deprecated
class LineUpAction extends SimpleAction implements DistanceAwayLinkedAction {
	public static final double MAX_SPEED = .3;
	private static final long FAIL_NOTIFY_TIME = 100;
	private static final long MAX_FAIL_TIME = 1000;
	/** The number of inches in front of the vision to target when the angle is too big.*/
	private static final double TARGET_IN_FRONT_INCHES = 0;
	
	private final VisionPacketProvider packetProvider;
	private final Supplier<SwerveDrive> driveSupplier;
	private final Supplier<Orientation> orientationSupplier;
	
	private final Action successAction;
	private final EventSender eventSender;
	
	private VisionView visionView = null;
	/** Used for sending sounds. Set to true once we found it. Set to false when we lose it.*/
	private boolean hasFound = false;
	private Action nextAction;
	private Long failureStartTime = null;
	private Long lastFailSound = null;
	
	private double distanceAway = Double.MAX_VALUE;
	
	LineUpAction(VisionPacketProvider packetProvider,
				 Supplier<SwerveDrive> driveSupplier, Supplier<Orientation> orientationSupplier,
				 Action failAction, Action successAction, EventSender eventSender) {
		super(false);
		this.packetProvider = Objects.requireNonNull(packetProvider);
		this.driveSupplier = Objects.requireNonNull(driveSupplier);
		this.orientationSupplier = Objects.requireNonNull(orientationSupplier);
		
		this.successAction = successAction;
		this.eventSender = eventSender;
		
		nextAction = failAction;
	}
	
	@Override
	protected void onUpdate() {
		super.onUpdate();
		final VisionPacket vision = packetProvider.getPacket();
		final boolean failed;
		if(vision != null){
			if(!hasFound){
				if(eventSender != null) {
					eventSender.sendEvent(SoundEvents.TARGET_FOUND);
				}
				hasFound = true;
			}
			failed = false;
			usePacket(vision);
		} else {
			failed = true;
		}
		if(failed){
			if(visionView != null){
				useVisionView(visionView);
			}
			if(failureStartTime == null){
				failureStartTime = System.currentTimeMillis();
			}
			if(failureStartTime + FAIL_NOTIFY_TIME < System.currentTimeMillis() && (lastFailSound == null || lastFailSound + 1000 < System.currentTimeMillis())){
				if(eventSender != null) {
					eventSender.sendEvent(SoundEvents.TARGET_FAILED);
				}
				lastFailSound = System.currentTimeMillis();
			}
			if(failureStartTime + MAX_FAIL_TIME < System.currentTimeMillis()){
				System.out.println("Failed vision. setDone(true) now");
				setDone(true);
			}
		} else {
			failureStartTime = null;
		}
	}
	private void updateVisionView(VisionPacket vision){
		final double orientation = orientationSupplier.get().getOrientationDegrees();
		final double visionYaw = vision.getVisionYaw() + orientation;
		
		final double direction = vision.getGroundAngle() + orientation;
		visionView = new VisionView(direction, vision.getGroundDistance(), visionYaw, driveSupplier.get());
	}
	private void usePacket(final VisionPacket targetVision){
		Objects.requireNonNull(targetVision);
		
		distanceAway = targetVision.getGroundDistance();
		final VisionPacket moveVision = new ImmutableVisionPacket(
				targetVision.getRobotX() * 1.7,
				targetVision.getRobotY(),
				targetVision.getRobotZ() + TARGET_IN_FRONT_INCHES, // make it closer
				targetVision.getVisionYaw(), targetVision.getVisionPitch(), targetVision.getVisionRoll(), targetVision.getImageX(), targetVision.getImageY()
		);
		final VisionPacket faceVision = new ImmutableVisionPacket(
				targetVision.getRobotX(),
				targetVision.getRobotY(),
				targetVision.getRobotZ() - 15, // make it further away
				targetVision.getVisionYaw(), targetVision.getVisionPitch(), targetVision.getVisionRoll(), targetVision.getImageX(), targetVision.getImageY()
		);
		updateVisionView(moveVision);
		final double groundDistance = moveVision.getGroundDistance();
//		if(groundDistance < 10){
//			final double percentage = groundDistance / 10;
//			xMult = VISION_PACKET_X_MULT * percentage + .9 * (1 - percentage);
//		} else {
//			xMult = VISION_PACKET_X_MULT;
//		}
		
		
		final double moveX = (moveVision.getVisionX() + 0); // + 6 is just to get it to line up better but really shouldn't be fixed here
		final double moveY = moveVision.getVisionZ();
		final double moveMagnitude = hypot(moveX, moveY);
		
		final double yawTurnAmount = max(-1, min(1, targetVision.getVisionYaw() / -30)); // moveVision has same yaw so it doesn't matter // to make the yaw go to 0
		final double zeroGroundAngle = MathUtil.minChange(faceVision.getGroundAngle(), 90, 360); // we want this to get close to 0 // we want to face the target
		final double faceTurnAmount = max(-1, min(1, zeroGroundAngle / -90)); // to face the target
		
//		final double turnAmount = .5 * max(-1, min(1,
//				max(-1, min(1, vision.getVisionYaw() / -30))
//						+ .5 * vision.getImageX()
//		));
		final double turnAmount;
//		if (groundDistance > 40) {
//			turnAmount = faceTurnAmount;
//		} else if (groundDistance > 20) {
//			final double percentage = (groundDistance - 20) / 20.0;
//			turnAmount = percentage * faceTurnAmount + (1 - percentage) * yawTurnAmount;
//		} else {
//			turnAmount = yawTurnAmount;
//		}
		turnAmount = faceTurnAmount * .5 + yawTurnAmount * .5;
		
		
		driveSupplier.get().setControl(new Vector2(moveX / moveMagnitude, moveY / moveMagnitude).rotateDegrees(-packetProvider.getPerspective().getForwardDirection()), turnAmount, MAX_SPEED);
		if(moveY < 5){
			nextAction = successAction;
			System.out.println("success! " + hashCode());
			setDone(true);
		}
	}
	private void useVisionView(VisionView visionView){
		Objects.requireNonNull(visionView);
//		SmartDashboard.putString("Vision Movement", "view");
		final double orientation = orientationSupplier.get().getOrientationDegrees();
		final double direction = visionView.getDirectionToTarget() - orientation; // forward is 90 degrees
		final double directionRadians = toRadians(direction);
		final double visionYaw = MathUtil.minChange(visionView.getTargetOrientation() - orientation, 0, 360); // perfect is 0 degrees
		final double distance = visionView.getDistanceToTarget();
		
		final double moveX = cos(directionRadians);
		final double moveY = sin(directionRadians);
		
		final double distanceLeft = distance - visionView.getTracker().calculateDistance();
		if(distanceLeft <= 10){
			System.out.println("Using vision view. We went the distance we needed to. " + hashCode());
			setDone(true);
		} else {
			driveSupplier.get().setControl(new Vector2(moveX, moveY).rotateDegrees(-packetProvider.getPerspective().getForwardDirection()), .5 * max(-1, min(1, visionYaw / -30)), MAX_SPEED);
		}
		distanceAway = max(0, distanceLeft + TARGET_IN_FRONT_INCHES);
	}
	
	@Override
	public Action getNextAction() {
		return nextAction;
	}
	
	@Override
	public double getInchesAway() {
		return distanceAway;
	}
}
