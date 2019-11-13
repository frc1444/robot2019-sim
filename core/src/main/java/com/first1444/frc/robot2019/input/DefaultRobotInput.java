package com.first1444.frc.robot2019.input;

import me.retrodaredevil.controller.SimpleControllerInput;
import me.retrodaredevil.controller.input.*;
import me.retrodaredevil.controller.output.ControllerRumble;
import me.retrodaredevil.controller.output.DisconnectedRumble;
import me.retrodaredevil.controller.types.ExtremeFlightJoystickControllerInput;
import me.retrodaredevil.controller.types.LogitechAttack3JoystickControllerInput;
import me.retrodaredevil.controller.types.RumbleCapableController;
import me.retrodaredevil.controller.types.StandardControllerInput;
import me.retrodaredevil.controller.input.implementations.*;

import java.util.function.Supplier;

/**
 * A class that takes care of all the controllers connected to the driver station and
 * exposes some of their inputs to be used in other parts of the program
 * <p>
 * This must be updated each call to period, and is usually done by a {@link me.retrodaredevil.controller.ControllerManager}
 */
public class DefaultRobotInput extends SimpleControllerInput implements RobotInput {
	private final StandardControllerInput controller;
	private final Supplier<ControllerRumble> rumbleSupplier;
	private final ExtremeFlightJoystickControllerInput operatorJoy;
	private final LogitechAttack3JoystickControllerInput climbJoy;

	private final InputPart movementSpeed;
	private final InputPart liftManualSpeed;
	private final InputPart cargoIntakeSpeed;
	private final InputPart hatchManualPivotSpeed;
	private final InputPart climbLiftSpeed;
	private final InputPart climbWheelSpeed;
	
	/**
	 * The passed controllers cannot have parents.
	 * @param controller
	 * @param operatorJoy
	 * @param climbJoy
	 * @param rumble The rumble or null. This CAN have a parent
	 */
	public DefaultRobotInput(StandardControllerInput controller,
							 ExtremeFlightJoystickControllerInput operatorJoy,
							 LogitechAttack3JoystickControllerInput climbJoy,
							 ControllerRumble rumble){
		this.controller = controller;
		this.operatorJoy = operatorJoy;
		this.climbJoy = climbJoy;
		if(rumble != null){
			partUpdater.addPartAssertNotPresent(rumble);
			this.rumbleSupplier = () -> rumble;
		} else {
			if (controller instanceof RumbleCapableController) {
				this.rumbleSupplier = ((RumbleCapableController) controller)::getRumble;
			} else {
				this.rumbleSupplier = () -> DisconnectedRumble.getInstance();
			}
		}
		partUpdater.addPartsAssertNonePresent(controller, operatorJoy, climbJoy); // add the controllers as children
		
		movementSpeed = new TwoWayInput(
				References.create(controller::getRightTrigger), // forward
				References.create(controller::getLeftTrigger) // backward
		);
		partUpdater.addPartAssertNotPresent(movementSpeed);
		liftManualSpeed = new MultiplierInputPart(
				References.create(() -> operatorJoy.getMainJoystick().getYAxis()), // analog full
				new HighestPositionInputPart(
						References.create(operatorJoy::getTrigger),
						References.create(this::getCargoLiftManualAllowed),
						References.create(this::getLiftManualOverrideAllowed)
				)
		);
		partUpdater.addPartAssertNotPresent(liftManualSpeed);
		cargoIntakeSpeed = new MultiplierInputPart(
				true,
				new ScaledInputPart(AxisType.ANALOG, References.create(operatorJoy::getSlider)), // analog
				References.create(() -> operatorJoy.getDPad().getYAxis()) // digital full
		);
		partUpdater.addPartAssertNotPresent(cargoIntakeSpeed);
		hatchManualPivotSpeed = new MultiplierInputPart(
				References.create(() -> climbJoy.getMainJoystick().getYAxis()), // analog full
				References.create(climbJoy::getThumbUpper) // digital
		);
		partUpdater.addPartAssertNotPresent(hatchManualPivotSpeed);
		climbLiftSpeed = new MultiplierInputPart(
				References.create(() -> climbJoy.getMainJoystick().getYAxis()),
				References.create(climbJoy::getTrigger)
		);
		partUpdater.addPartAssertNotPresent(climbLiftSpeed);
		climbWheelSpeed = new MultiplierInputPart(
				References.create(() -> climbJoy.getMainJoystick().getYAxis()),
				References.create(climbJoy::getThumbLower)
		);
		partUpdater.addPartAssertNotPresent(climbWheelSpeed);
	}

	// region Drive Controls
	@Override
	public JoystickPart getMovementJoy() {
		return controller.getLeftJoy();
	}

	@Override
	public InputPart getTurnAmount() {
		return controller.getRightJoy().getXAxis();
	}

	@Override
	public InputPart getMovementSpeed() {
		return movementSpeed;
	}
	// endregion
	
	
	@Override
	public InputPart getVisionAlign() {
		return controller.getRightBumper();
	}
	
	@Override
	public InputPart getFirstPersonHoldButton() {
		return controller.getLeftBumper();
	}
	
	@Override
	public InputPart getLiftManualSpeed() {
		return liftManualSpeed;
	}
	
	@Override
	public InputPart getCargoLiftManualAllowed() {
		return operatorJoy.getGridUpperRight();
	}
	
	@Override
	public InputPart getLiftManualOverrideAllowed() {
		return climbJoy.getLeftLower();
	}
	
	@Override
	public InputPart getCargoIntakeSpeed() {
		return cargoIntakeSpeed;
	}
	
	@Override
	public InputPart getHatchManualPivotSpeed() {
		return hatchManualPivotSpeed;
	}
	// region Hatch Pivot Presets
	@Override
	public InputPart getHatchPivotGroundPreset() {
		return operatorJoy.getThumbButton();
	}
	@Override
	public InputPart getHatchPivotReadyPreset() {
		return operatorJoy.getThumbLeftLower();
	}
	@Override
	public InputPart getHatchPivotStowedPreset() {
		return operatorJoy.getThumbRightLower();
	}
	// endregion
	
	
	@Override
	public InputPart getHatchDrop() {
		return operatorJoy.getThumbRightUpper();
	}
	
	@Override
	public InputPart getHatchGrab() {
		return operatorJoy.getThumbLeftUpper();
	}
	
	// region Lift Presets
	@Override
	public InputPart getLevel1Preset() { // final
		return operatorJoy.getGridLowerLeft();
	}
	@Override
	public InputPart getLevel2Preset() { // final
		return operatorJoy.getGridMiddleLeft();
	}
	@Override
	public InputPart getLevel3Preset() { // final
		return operatorJoy.getGridUpperLeft();
	}
	@Override
	public InputPart getLevelCargoShipCargoPreset() { // final
		return operatorJoy.getGridMiddleRight();
	}
	@Override
	public InputPart getCargoPickupPreset() { // final
		return operatorJoy.getGridLowerRight();
	}
	// endregion
	
	
	@Override
	public InputPart getDefenseButton() {
		return climbJoy.getRightUpper();
	}
	
	@Override
	public InputPart getClimbLiftSpeed() {
		return climbLiftSpeed;
	}
	
	@Override
	public InputPart getClimbWheelSpeed() {
		return climbWheelSpeed;
	}
	
	@Override
	public InputPart getAutonomousCancelButton() {
		return controller.getLeftStick();
	}
	
	@Override
	public JoystickPart getResetGyroJoy() {
		return controller.getDPad();
	}
	
	@Override
	public ControllerRumble getDriverRumble() {
		return rumbleSupplier.get();
	}
	
	@Override
	public InputPart getAutonomousWaitButton() {
		return climbJoy.getLeftLower();
	}
	
	@Override
	public InputPart getAutonomousStartButton() {
		return climbJoy.getLeftUpper();
	}
	
	@Override
	public InputPart getSwerveQuickReverseCancel() {
		return controller.getSelect();
	}
	
	@Override
	public InputPart getSwerveRecalibrate() {
		return controller.getStart();
	}
	
	@Override
	public InputPart getCameraToggleButton() {
		return controller.getFaceRight();
	}
	
	@Override
	public boolean isConnected() {
		return controller.isConnected() && operatorJoy.isConnected();
	}
}
