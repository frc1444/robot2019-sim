package com.first1444.frc.robot2019.actions;

import com.first1444.frc.robot2019.Constants;
import com.first1444.frc.robot2019.Perspective;
import com.first1444.frc.robot2019.RobotDimensions;
import com.first1444.frc.robot2019.autonomous.actions.vision.LineUpCreator;
import com.first1444.frc.robot2019.input.RobotInput;
import com.first1444.frc.robot2019.subsystems.TaskSystem;
import com.first1444.frc.robot2019.vision.BestVisionPacketSelector;
import com.first1444.frc.robot2019.vision.DefaultVisionPacketProvider;
import com.first1444.frc.robot2019.vision.VisionSupplier;
import com.first1444.frc.util.MathUtil;
import com.first1444.sim.api.Vector2;
import com.first1444.sim.api.drivetrain.swerve.SwerveDrive;
import com.first1444.sim.api.sensors.Orientation;
import me.retrodaredevil.action.*;
import me.retrodaredevil.controller.input.InputPart;
import me.retrodaredevil.controller.input.JoystickPart;
import me.retrodaredevil.controller.output.ControllerRumble;

import java.util.Objects;
import java.util.function.Supplier;

/**
 * This swerve controls for teleop and should be ended when teleop is over. This can be recycled
 */
public class SwerveDriveAction extends SimpleAction {
	private final Supplier<SwerveDrive> driveSupplier;
	private final Supplier<Orientation> orientationSupplier;
	private final Supplier<TaskSystem> taskSystemSupplier;
	private final RobotInput input;
	private final ActionChooser actionChooser;
	private final VisionSupplier visionSupplier;
	private final RobotDimensions dimensions;
	
	/** The perspective or null to automatically choose the perspective based on the task */
	private Perspective perspective = Perspective.DRIVER_STATION;
	
	public SwerveDriveAction(Supplier<SwerveDrive> driveSupplier, Supplier<Orientation> orientationSupplier, Supplier<TaskSystem> taskSystemSupplier, RobotInput input, VisionSupplier visionSupplier, RobotDimensions dimensions) {
		super(true);
		this.driveSupplier = Objects.requireNonNull(driveSupplier);
		this.orientationSupplier = Objects.requireNonNull(orientationSupplier);
		this.taskSystemSupplier = Objects.requireNonNull(taskSystemSupplier);
		this.input = Objects.requireNonNull(input);
		this.actionChooser = Actions.createActionChooserRecyclable(WhenDone.BE_DONE);
		this.visionSupplier = visionSupplier;
		this.dimensions = dimensions;
	}
	
	/**
	 *
	 * @param perspective The perspective or null to automatically choose the perspective based on the task
	 */
	public void setPerspective(Perspective perspective){
		this.perspective = perspective;
	}

	@Override
	protected void onStart() {
		super.onStart();
		final ControllerRumble rumble = input.getDriverRumble();
		if(rumble.isConnected()){
			rumble.rumbleTime(250, .4);
		}
	}

	@Override
	protected void onUpdate() {
		super.onUpdate();
		final Perspective perspective;
		final TaskSystem.Task task = taskSystemSupplier.get().getCurrentTask();
		if(input.getFirstPersonHoldButton().isDown() || this.perspective == null){
			perspective = task == TaskSystem.Task.HATCH
					? dimensions.getHatchManipulatorPerspective()
					: dimensions.getCargoManipulatorPerspective();
		} else {
			perspective = this.perspective;
		}
		Objects.requireNonNull(perspective);
		
		if(input.getVisionAlign().isDown()){
			if(input.getVisionAlign().isJustPressed()){
				actionChooser.setNextAction(LineUpCreator.createLineUpAction(
						new DefaultVisionPacketProvider(
								task == TaskSystem.Task.CARGO ? dimensions.getCargoManipulatorPerspective() : dimensions.getHatchManipulatorPerspective(),
								visionSupplier,
								task == TaskSystem.Task.CARGO ? dimensions.getCargoCameraID() : dimensions.getHatchCameraID(),
								new BestVisionPacketSelector(),
								Constants.VISION_PACKET_VALIDITY_TIME
						),
						driveSupplier, orientationSupplier,  null, null, null
				));
			}
			actionChooser.update();
			if(actionChooser.isDone()){
				final ControllerRumble rumble = input.getDriverRumble();
				if(rumble.isConnected()) {
					rumble.rumbleTimeout(200, .3);
				}
			}
		} else {
			if(actionChooser.isActive()){
				actionChooser.end();
			}
			final SwerveDrive drive = Objects.requireNonNull(driveSupplier.get());
			
			final JoystickPart joystick = input.getMovementJoy();
			final double x, y;
			if(joystick.isDeadzone()){
				x = 0;
				y = 0;
			} else {
				x = joystick.getCorrectX();
				y = joystick.getCorrectY();
			}

			final double turnAmount;
			if(input.getTurnAmount().isDeadzone()){
				turnAmount = 0;
			} else {
				turnAmount = input.getTurnAmount().getPosition();
			}
			
			final InputPart speedInputPart = input.getMovementSpeed();
			final double speed;
			if (speedInputPart.isDeadzone()) {
				speed = 0;
			} else {
				speed = MathUtil.conservePow(speedInputPart.getPosition(), 2);
			}
			final Vector2 translation;
			double offsetRadians = perspective.getOffsetRadians();
			double orientationRadians = orientationSupplier.get().getOrientationRadians();
			if(perspective.isUseGyro()){
				translation = new Vector2(x, y).rotateRadians(-orientationRadians - offsetRadians);
			} else {
				translation = new Vector2(y, -x).rotateRadians(-offsetRadians);
			}

			drive.setControl(translation, turnAmount, speed);
		}
	}

}
