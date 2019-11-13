package com.first1444.frc.robot2019.autonomous;

import com.first1444.frc.robot2019.Constants;
import com.first1444.frc.robot2019.Robot;
import com.first1444.frc.robot2019.autonomous.actions.*;
import com.first1444.frc.robot2019.autonomous.actions.vision.LineUpCreator;
import com.first1444.frc.robot2019.deepspace.SlotLevel;
import com.first1444.frc.robot2019.subsystems.Lift;
import com.first1444.frc.robot2019.vision.BestVisionPacketSelector;
import com.first1444.frc.robot2019.vision.DefaultVisionPacketProvider;
import edu.wpi.first.wpilibj.DriverStation;
import me.retrodaredevil.action.Action;
import me.retrodaredevil.action.Actions;
import me.retrodaredevil.action.SimpleAction;
import me.retrodaredevil.action.WhenDone;

import java.util.Map;

public class RobotAutonActionCreator implements AutonActionCreator {
	private static final Map<SlotLevel, Lift.Position> SLOT_MAP = Map.of(
			SlotLevel.LEVEL1, Lift.Position.LEVEL1,
			SlotLevel.LEVEL2, Lift.Position.LEVEL2,
			SlotLevel.LEVEL3, Lift.Position.LEVEL3
	);
	private final Robot robot;

	public RobotAutonActionCreator(Robot robot) {
		this.robot = robot;
	}
	
	@Override
	public Action createLogMessageAction(String message) {
		return Actions.createRunOnce(() -> System.out.println(message));
	}
	
	@Override
	public Action createLogWarningAction(String message) {
		return Actions.createRunOnce(() -> DriverStation.reportWarning(message, false));
	}
	
	@Override
	public Action createLogErrorAction(String message) {
		return Actions.createRunOnce(() -> DriverStation.reportError(message, false));
	}
	
	@Override
	public Action createTurnToOrientation(double desiredOrientation) {
		return new TurnToOrientation(desiredOrientation, robot::getDrive, robot::getOrientation);
	}

	@Override
	public Action createGoStraight(double distanceInches, double speed, double angleDegrees) {
		return GoStraight.createGoStraightAtHeading(distanceInches, speed, angleDegrees, null, robot::getDrive, robot::getOrientation);
	}

	@Override
	public Action createGoStraight(double distanceInches, double speed, double angleDegrees, double faceDirectionDegrees) {
		return GoStraight.createGoStraightAtHeading(distanceInches, speed, angleDegrees, faceDirectionDegrees, robot::getDrive, robot::getOrientation);
	}

	@Override
	public Action createCargoShipPlaceHatchUseVision(Action failAction, Action successAction) {
		return createRocketPlaceHatchUseVision(SlotLevel.LEVEL1, failAction, successAction);
	}

	@Override
	public Action createCargoShipPlaceCargoUseVision(Action failAction, Action successAction) {
		return createLineUpWithRunner(true, Lift.Position.CARGO_CARGO_SHIP, failAction, successAction);
	}
	
	@Override
	public Action createRocketPlaceCargoUseVision(SlotLevel slotLevel, Action failAction, Action successAction) {
		return createLineUpWithRunner(false, SLOT_MAP.get(slotLevel), failAction, successAction);
	}
	
	@Override
	public Action createRocketPlaceHatchUseVision(SlotLevel slotLevel, Action failAction, Action successAction) {
		return createLineUpWithRunner(true, SLOT_MAP.get(slotLevel), failAction, successAction);
	}
	private Action createLineUpWithRunner(boolean hatch, Lift.Position liftPosition, Action failAction, Action successAction){
		final boolean[] success = {false};
		final boolean[] fail = {false};
		
		final var lineUp = LineUpCreator.createLinkedLineUpAction(
				new DefaultVisionPacketProvider(
						hatch ? robot.getDimensions().getHatchManipulatorPerspective() : robot.getDimensions().getCargoManipulatorPerspective(),
						robot.getVisionSupplier(),
						hatch ? robot.getDimensions().getHatchCameraID() : robot.getDimensions().getCargoCameraID(),
						new BestVisionPacketSelector(),
						Constants.VISION_PACKET_VALIDITY_TIME
				),
				robot::getDrive, robot::getOrientation,
				Actions.createRunOnce(() -> fail[0] = true),
				Actions.createRunOnce(() -> success[0] = true),
				robot.getSoundSender()
		);
		final boolean[] finalActionSuccess = {false};
		return new Actions.ActionQueueBuilder(
				new SimpleAction(false){
					final Action lineUpRunner = Actions.createLinkedActionRunner(lineUp, WhenDone.CLEAR_ACTIVE_AND_BE_DONE, true);
					@Override
					protected void onUpdate() {
						super.onUpdate();
						lineUpRunner.update();
						final double distanceLeft;
						if(lineUp.isActive()){
							distanceLeft = lineUp.getInchesAway();
						} else if(success[0]){
							distanceLeft = 0;
						} else if(fail[0]){
							setDone(true);
							return;
						} else {
							System.err.println("I didn't expect this to happen, but I prepared for it anyway");
							distanceLeft = Double.MAX_VALUE;
						}
						robot.getCargoIntake().stow(); // always stow cargo intake
						final Lift lift = robot.getLift();
						if(distanceLeft < 40){
							lift.setDesiredPosition(liftPosition);
							if(hatch){
								robot.getHatchIntake().readyPosition();
							}
						}
						if(lineUpRunner.isDone() && lift.isDesiredPositionReached()){
							finalActionSuccess[0] = true;
							setDone(true);
						}
					}
				},
				Actions.createDynamicActionRunner(() -> {
					final Action action;
					if(finalActionSuccess[0]){
						final Action releaseAction = hatch ? createDropHatch() : createReleaseCargo();
						action = Actions.createLinkedAction(releaseAction, successAction);
					} else {
						action = failAction;
					}
					return Actions.createLinkedActionRunner(action, WhenDone.CLEAR_ACTIVE_AND_BE_DONE, true);
				})
		).immediatelyDoNextWhenDone(true).build();
	}
	
	@Override
	public Action createExtendHatch() {
		return HatchPositionAction.createReady(robot::getHatchIntake);
	}
	
	@Override
	public Action createStowHatch() {
		return HatchPositionAction.createStow(robot::getHatchIntake);
	}
	
	@Override
	public Action createDropHatch() {
		return HatchIntakeAction.createDrop(robot::getHatchIntake);
	}
	
	@Override
	public Action createGrabHatch() {
		return HatchIntakeAction.createGrab(robot::getHatchIntake);
	}
	
	@Override
	public Action createReleaseCargo() {
		return new TimedCargoIntake(500, robot::getCargoIntake, 1);
	}
	
	@Override
	public Action createRaiseLift(Lift.Position position) {
		return new RaiseLift(robot::getLift, position);
	}
}
