package com.first1444.frc.robot2019.actions;

import com.first1444.frc.robot2019.Perspective;
import com.first1444.frc.robot2019.RobotDimensions;
import com.first1444.frc.robot2019.autonomous.actions.vision.LineUpCreator;
import com.first1444.frc.robot2019.input.RobotInput;
import com.first1444.frc.robot2019.sound.SilentSoundMap;
import com.first1444.frc.robot2019.subsystems.TaskSystem;
import com.first1444.sim.api.Clock;
import com.first1444.sim.api.Vector2;
import com.first1444.sim.api.drivetrain.swerve.SwerveDrive;
import com.first1444.sim.api.sensors.Orientation;
import com.first1444.sim.api.surroundings.SurroundingProvider;
import me.retrodaredevil.action.ActionChooser;
import me.retrodaredevil.action.Actions;
import me.retrodaredevil.action.SimpleAction;
import me.retrodaredevil.action.WhenDone;
import me.retrodaredevil.controller.input.InputPart;
import me.retrodaredevil.controller.input.JoystickPart;
import me.retrodaredevil.controller.output.ControllerRumble;

import static com.first1444.sim.api.MathUtil.conservePow;
import static java.util.Objects.requireNonNull;

/**
 * This swerve controls for teleop and should be ended when teleop is over. This can be recycled
 */
public class SwerveDriveAction extends SimpleAction {
    private final Clock clock;
    private final SwerveDrive drive;
    private final Orientation orientation;
    private final TaskSystem taskSystem;
    private final RobotInput input;
    private final SurroundingProvider surroundingProvider;
    private final RobotDimensions dimensions;


    private final ActionChooser actionChooser;

    /** The perspective or null to automatically choose the perspective based on the task */
    private Perspective perspective = Perspective.DRIVER_STATION;

    public SwerveDriveAction(
            Clock clock,
            SwerveDrive drive, Orientation orientation, TaskSystem taskSystem,
            RobotInput input,
            SurroundingProvider surroundingProvider,
            RobotDimensions dimensions) {
        super(true);
        this.clock = requireNonNull(clock);
        this.drive = requireNonNull(drive);
        this.orientation = requireNonNull(orientation);
        this.taskSystem = requireNonNull(taskSystem);
        this.input = requireNonNull(input);
        this.surroundingProvider = requireNonNull(surroundingProvider);
        this.dimensions = requireNonNull(dimensions);

        actionChooser = Actions.createActionChooserRecyclable(WhenDone.BE_DONE);
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
        final TaskSystem.Task task = taskSystem.getCurrentTask();
        if(input.getFirstPersonHoldButton().isDown() || this.perspective == null){
            perspective = task == TaskSystem.Task.HATCH
                    ? dimensions.getHatchManipulatorPerspective()
                    : dimensions.getCargoManipulatorPerspective();
        } else {
            perspective = this.perspective;
        }
        requireNonNull(perspective);

        if(input.getVisionAlign().isDown()){
            if(input.getVisionAlign().isJustPressed()){
                actionChooser.setNextAction(LineUpCreator.createLineUpAction(
                        clock, surroundingProvider,
                        drive, orientation,
                        task == TaskSystem.Task.HATCH
                                ? dimensions.getHatchManipulatorPerspective().getOffset()
                                : dimensions.getCargoManipulatorPerspective().getOffset(),
                        null, null, SilentSoundMap.INSTANCE
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
                speed = conservePow(speedInputPart.getPosition(), 2);
            }
            final Vector2 translation;
            double offsetRadians = perspective.getOffsetRadians();
            double orientationRadians = orientation.getOrientationRadians();
            if(perspective.isUseGyro()){
                translation = new Vector2(x, y).rotateRadians(offsetRadians - orientationRadians);
            } else {
                //noinspection SuspiciousNameCombination
                translation = new Vector2(y, -x).rotateRadians(offsetRadians);
            }

            drive.setControl(translation, turnAmount, speed);
        }
    }

}
