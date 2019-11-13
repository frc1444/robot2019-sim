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
import me.retrodaredevil.action.Action;
import me.retrodaredevil.action.SimpleAction;

import java.util.Objects;
import java.util.function.Supplier;

import static java.lang.Math.*;

public class StrafeLineUpAction extends SimpleAction implements DistanceAwayLinkedAction {
	public static final double MAX_SPEED = .3;
	private static final long FAIL_NOTIFY_TIME = 100;
	private static final long MAX_FAIL_TIME = 2000;
	private static final double MAX_MOVE_X = 5;
	
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
	
	StrafeLineUpAction(VisionPacketProvider packetProvider,
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
		if(vision != null && vision.getRobotZ() <= 0){
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
		updateVisionView(targetVision);
		
		final VisionPacket faceVision = new ImmutableVisionPacket(
				targetVision.getRobotX(),
				targetVision.getRobotY(),
				targetVision.getRobotZ() - 30, // make it further away
				targetVision.getVisionYaw(), targetVision.getVisionPitch(), targetVision.getVisionRoll(), targetVision.getImageX(), targetVision.getImageY()
		);
		final double yawTurnAmount = max(-1, min(1, targetVision.getVisionYaw() / -30)); // moveVision has same yaw so it doesn't matter // to make the yaw go to 0
		final double zeroGroundAngle = MathUtil.minChange(faceVision.getGroundAngle(), 90, 360); // we want this to get close to 0 // we want to face the target
		final double faceTurnAmount = max(-1, min(1, zeroGroundAngle / -70)); // to face the target
		final double turnAmount = faceTurnAmount * .6 + yawTurnAmount * .4;
		
//		final double moveY = min(1, Math.pow(40 / -targetVision.getRobotZ(), .5)); // go from 0 up to 1 as it gets close
		final double moveY = 1;
		final double moveX = max(-MAX_MOVE_X, min(MAX_MOVE_X, (targetVision.getRobotX()) / -30)) * (abs(targetVision.getGroundDistance() < 30 ? 1.4 : 1));
		final double moveMagnitude = hypot(moveX, moveY);
		if(targetVision.getRobotZ() >= -3 && abs(moveX) < 6){
			nextAction = successAction;
			System.out.println("strafe success!");
//			driveSupplier.get().setControl(0, 0, 0, 0, packetProvider.getPerspective());
			setDone(true);
		} else {
			driveSupplier.get().setControl(new Vector2(moveX / moveMagnitude, moveY / moveMagnitude).rotateDegrees(-packetProvider.getPerspective().getForwardDirection()), turnAmount, MAX_SPEED);
		}
	}
	private void useVisionView(VisionView visionView){
		Objects.requireNonNull(visionView);
		final double orientation = orientationSupplier.get().getOrientationDegrees();
		final double direction = visionView.getDirectionToTarget() - orientation; // forward is 90 degrees
		final double directionRadians = toRadians(direction);
		final double visionYaw = MathUtil.minChange(visionView.getTargetOrientation() - orientation, 0, 360); // perfect is 0 degrees
		final double distance = visionView.getDistanceToTarget();
		
		final double moveX = cos(directionRadians);
		final double moveY = sin(directionRadians);
		
		final double distanceLeft = distance - visionView.getTracker().calculateDistance();
		if(distanceLeft > 30){
			System.out.println("We must have lost vision. Using distanceLeft with more than 30 inches");
		}
		if(distanceLeft <= 5){
			nextAction = successAction;
			System.out.println("Using vision view. We went the distance we needed to");
			setDone(true);
		} else {
			driveSupplier.get().setControl(new Vector2(moveX, moveY).rotateDegrees(-packetProvider.getPerspective().getForwardDirection()), .5 * max(-1, min(1, visionYaw / -30)), MAX_SPEED);
		}
		distanceAway = max(0, distanceLeft);
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
