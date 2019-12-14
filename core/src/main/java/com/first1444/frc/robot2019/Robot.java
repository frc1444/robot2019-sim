/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                                                */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package com.first1444.frc.robot2019;

import com.first1444.dashboard.advanced.implementations.chooser.ChooserSendable;
import com.first1444.dashboard.advanced.implementations.chooser.MutableMappedChooserProvider;
import com.first1444.dashboard.advanced.implementations.chooser.SimpleMappedChooserProvider;
import com.first1444.dashboard.shuffleboard.ComponentMetadataHelper;
import com.first1444.dashboard.shuffleboard.SendableComponent;
import com.first1444.frc.robot2019.actions.OperatorAction;
import com.first1444.frc.robot2019.actions.SwerveCalibrateAction;
import com.first1444.frc.robot2019.actions.SwerveDriveAction;
import com.first1444.frc.robot2019.autonomous.AutonomousChooserState;
import com.first1444.frc.robot2019.autonomous.original.OriginalAutonomousModeCreator;
import com.first1444.frc.robot2019.autonomous.original.RobotOriginalAutonActionCreator;
import com.first1444.frc.robot2019.autonomous.actions.TimedCargoIntake;
import com.first1444.frc.robot2019.input.DefaultRobotInput;
import com.first1444.frc.robot2019.input.InputUtil;
import com.first1444.frc.robot2019.input.RobotInput;
import com.first1444.frc.robot2019.sound.DefaultSoundMap;
import com.first1444.frc.robot2019.sound.SoundMap;
import com.first1444.frc.robot2019.subsystems.*;
import com.first1444.frc.robot2019.subsystems.implementations.DefaultTaskSystem;
import com.first1444.frc.robot2019.vision.VisionPacketListener;
import com.first1444.sim.api.Clock;
import com.first1444.sim.api.distance.*;
import com.first1444.sim.api.drivetrain.swerve.FourWheelSwerveDrive;
import com.first1444.sim.api.drivetrain.swerve.FourWheelSwerveDriveData;
import com.first1444.sim.api.drivetrain.swerve.SwerveDrive;
import com.first1444.sim.api.frc.AdvancedIterativeRobotAdapter;
import com.first1444.sim.api.frc.FrcDriverStation;
import com.first1444.sim.api.frc.FrcLogger;
import com.first1444.sim.api.frc.FrcMode;
import com.first1444.sim.api.scheduler.match.DefaultMatchScheduler;
import com.first1444.sim.api.scheduler.match.MatchScheduler;
import com.first1444.sim.api.scheduler.match.MatchTime;
import com.first1444.sim.api.sensors.Orientation;
import com.first1444.sim.api.sensors.OrientationHandler;
import com.first1444.sim.api.sound.SoundCreator;
import com.first1444.sim.api.surroundings.SurroundingProvider;
import me.retrodaredevil.action.*;
import me.retrodaredevil.controller.ControlConfig;
import me.retrodaredevil.controller.MutableControlConfig;
import me.retrodaredevil.controller.PartUpdater;
import me.retrodaredevil.controller.implementations.ControllerPartCreator;
import me.retrodaredevil.controller.output.ControllerRumble;
import me.retrodaredevil.controller.types.StandardControllerInput;
import org.jetbrains.annotations.Nullable;

import java.util.HashMap;
import java.util.Map;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends AdvancedIterativeRobotAdapter {
    private static final ControlConfig controlConfig;
    static {
        MutableControlConfig config = new MutableControlConfig();
        // *edit values of controlConfig if desired*
        config.switchToSquareInputThreshold = 1.2;
        config.fullAnalogDeadzone = .075;
        config.analogDeadzone = .02;
        config.cacheAngleAndMagnitudeInUpdate = false;
        config.useAbstractedIsDownIfPossible = false; // On PS4 controllers, this option is too sensitive
        controlConfig = config;
    }
    private final FrcDriverStation driverStation;
    private final FrcLogger logger;
    private final Clock clock;
    private final DashboardMap dashboardMap;
    private final Lift lift;
    private final CargoIntake cargoIntake;
    private final HatchIntake hatchIntake;
    private final Climber climber;
    private final SurroundingProvider surroundingProvider;
    private final OrientationSystem orientationSystem;
    private final SwerveDrive drive;
    private final RobotDimensions dimensions;

    private final PartUpdater partUpdater = new PartUpdater();
    private final RobotInput robotInput;
    /** The distance accumulator where the position will never jump. This should be updated using {@link DistanceAccumulator#run()} */
    private final DistanceAccumulator relativeDistanceAccumulator;
    /** The distance accumulator representing the absolute position of the robot. This may jump around as we correct the position over time. This does not have to be updated. */
    private final MutableDistanceAccumulator absoluteDistanceAccumulator;

    private final MutableMappedChooserProvider<Perspective> autonomousPerspectiveChooser;
    private final TaskSystem taskSystem;
    private final MatchScheduler matchScheduler;

    private final VisionPacketListener visionPacketListener;
    private final SoundMap soundMap;

    /** An {@link Action} that updates certain subsystems only when the robot is enabled*/
    private final Action enabledSubsystemUpdater;
    /** An {@link Action} that updates certain subsystems all the time. If {@link #enabledSubsystemUpdater} is updated, this is updated after that*/
    private final Action constantSubsystemUpdater;
    /** The {@link ActionChooser} that handles an action that updates subsystems. (One action is active)*/
    private final ActionChooser actionChooser;
    private final ActionMultiplexer dynamicUpdater;

    private final Action teleopAction;
    private final SwerveDriveAction swerveDriveAction;
    private final AutonomousChooserState autonomousChooserState;

    // region Initialize
    /** Used to initialize final fields.*/
    public Robot(
            FrcDriverStation driverStation,
            FrcLogger logger,
            Clock clock,
            DashboardMap dashboardMap,
            StandardControllerInput controller, ControllerPartCreator port1, ControllerPartCreator port2, ControllerRumble rumble,
            SoundCreator soundCreator,
            OrientationHandler rawOrientationHandler,
            FourWheelSwerveDriveData fourWheelSwerveData,
            Lift lift, CargoIntake cargoIntake, HatchIntake hatchIntake, Climber climber,
            SurroundingProvider surroundingProvider,
            Action extraAction
    ){
        this.driverStation = driverStation;
        this.logger = logger;
        this.clock = clock;
        this.dashboardMap = dashboardMap;
        this.cargoIntake = cargoIntake;
        this.climber = climber;
        this.hatchIntake = hatchIntake;
        this.lift = lift;
        this.surroundingProvider = surroundingProvider;

        robotInput = new DefaultRobotInput(
                controller,
                InputUtil.createJoystick(port1),
                InputUtil.createAttackJoystick(port2),
                rumble
        );
        partUpdater.addPartAssertNotPresent(robotInput);
        partUpdater.updateParts(controlConfig); // update this so when calling get methods don't throw exceptions

        soundMap = new DefaultSoundMap(soundCreator);

        orientationSystem = new OrientationSystem(dashboardMap, rawOrientationHandler, robotInput);

        this.drive = new FourWheelSwerveDrive(fourWheelSwerveData);
        final DefaultTaskSystem defaultTaskSystem = new DefaultTaskSystem(robotInput);
        this.taskSystem = defaultTaskSystem;
        final DefaultMatchScheduler defaultMatchScheduler = new DefaultMatchScheduler(driverStation, clock);
        this.matchScheduler = defaultMatchScheduler;

        final MutableMappedChooserProvider<Boolean> cameraIDSwitchedChooser = new SimpleMappedChooserProvider<>();
        final Map<String, Boolean> cameraIDSwitchedMap = Map.of(
                "NOT FLIPPED", false,
                "FLIPPED", true
        );
        cameraIDSwitchedChooser.set(cameraIDSwitchedMap, "NOT FLIPPED");
        dashboardMap.getDevTab().add("Camera IDs Flipped", new SendableComponent<>(new ChooserSendable(cameraIDSwitchedChooser)));
        dimensions = new DynamicRobotDimensions(Constants.Dimensions.INSTANCE, cameraIDSwitchedChooser::getSelected);

        relativeDistanceAccumulator = new DeltaDistanceAccumulator(new OrientationDeltaDistanceCalculator(new SwerveDeltaDistanceCalculator(fourWheelSwerveData), orientationSystem.getOrientation()));
        absoluteDistanceAccumulator = new DefaultMutableDistanceAccumulator(relativeDistanceAccumulator, false);

        autonomousPerspectiveChooser = new SimpleMappedChooserProvider<>();
        Map<String, Perspective> autonomousPerspectiveMap = new HashMap<>(Map.of(
                "Hatch Cam", dimensions.getHatchManipulatorPerspective(),
                "Cargo Cam", dimensions.getCargoManipulatorPerspective(),
                "Driver Station (blind field centric)", Perspective.DRIVER_STATION,
                "Jumbotron on Right", Perspective.JUMBOTRON_ON_RIGHT,
                "Jumbotron on Left", Perspective.JUMBOTRON_ON_LEFT
        ));
        autonomousPerspectiveMap.put("Auto Cam", null);
        autonomousPerspectiveChooser.set(autonomousPerspectiveMap, "Auto Cam");
        dashboardMap.getUserTab().add("Autonomous Perspective", new SendableComponent<>(new ChooserSendable(autonomousPerspectiveChooser)), (metadata) -> new ComponentMetadataHelper(metadata)
                .setSize(2, 1)
                .setPosition(9, 4));

        visionPacketListener = new VisionPacketListener(
                clock,
                Map.of(
                        dimensions.getHatchCameraID(), dimensions.getHatchManipulatorPerspective().getOffset(),
                        dimensions.getCargoCameraID(), dimensions.getCargoManipulatorPerspective().getOffset()
                ),
                "10.14.44.5", 5801
        );

        enabledSubsystemUpdater = new Actions.ActionMultiplexerBuilder(
                lift, cargoIntake, climber, hatchIntake
        ).clearAllOnEnd(false).canRecycle(true).build();

        constantSubsystemUpdater = new Actions.ActionMultiplexerBuilder( // NOTE, without forceUpdateInOrder(true), these will not update in order
                Actions.createRunForeverRecyclable(drive),
                Actions.createRunForeverRecyclable(relativeDistanceAccumulator),
                orientationSystem,
                defaultTaskSystem,
                Actions.createRunForever(defaultMatchScheduler),
                new SwerveCalibrateAction(drive, robotInput),
                extraAction
        ).clearAllOnEnd(false).canRecycle(false).build();
        actionChooser = Actions.createActionChooser(WhenDone.CLEAR_ACTIVE);
        dynamicUpdater = new Actions.ActionMultiplexerBuilder().forceUpdateInOrder(true).canRecycle(false).canBeDone(false).build();

        swerveDriveAction = new SwerveDriveAction(clock, drive, getOrientation(), taskSystem, robotInput, surroundingProvider, getDimensions());
        teleopAction = new Actions.ActionMultiplexerBuilder(
                swerveDriveAction,
                new OperatorAction(this, robotInput)
        ).clearAllOnEnd(false).canBeDone(false).canRecycle(true).build();
        autonomousChooserState = new AutonomousChooserState(
                dashboardMap, // this will add stuff to the dashboard
                clock,
                new OriginalAutonomousModeCreator(new RobotOriginalAutonActionCreator(this), dimensions),
                robotInput
        );

        visionPacketListener.start();
        System.out.println("Finished constructor");
    }

    @Override
    public void close() {
        visionPacketListener.interrupt();
        System.out.println("close() method called! Robot program must be ending!");
    }


    // endregion

    // region Overridden Methods

    /** Called before every other period method no matter what state the robot is in*/
    @Override
    public void robotPeriodic() {
        partUpdater.updateParts(controlConfig);

        actionChooser.update(); // update Actions that control the subsystems
        dynamicUpdater.update();

        if(driverStation.isEnabled()){
            enabledSubsystemUpdater.update(); // update subsystems when robot is enabled
        }
        constantSubsystemUpdater.update(); // update subsystems that are always updated
    }

    @Override
    public void disabledInit(@Nullable FrcMode previousMode) {
        dashboardMap.getLiveWindow().setEnabled(false);
        actionChooser.setToClearAction();
        if(enabledSubsystemUpdater.isActive()) {
            enabledSubsystemUpdater.end();
        }
        if(previousMode == FrcMode.TELEOP){
            soundMap.getMatchEnd().play();
        } else {
            soundMap.getDisable().play();
        }
    }

    @Override
    public void teleopInit() {
        actionChooser.setNextAction(new Actions.ActionMultiplexerBuilder(
                teleopAction
        ).canRecycle(false).canBeDone(true).build());
        swerveDriveAction.setPerspective(Perspective.DRIVER_STATION);
        soundMap.getTeleopEnable().play();
        matchScheduler.schedule(new MatchTime(1.2, MatchTime.Mode.TELEOP, MatchTime.Type.FROM_END), () -> {
            System.out.println("Stowing cargo intake");
            cargoIntake.stow();
        });
        matchScheduler.schedule(new MatchTime(.5, MatchTime.Mode.TELEOP, MatchTime.Type.FROM_END), () -> {
            dynamicUpdater.add(new TimedCargoIntake(clock, .4, cargoIntake, 1));
        });
        matchScheduler.schedule(new MatchTime(.5, MatchTime.Mode.TELEOP, MatchTime.Type.FROM_END), () -> {
            System.out.println("Dropping hatch");
            hatchIntake.drop();
        });
        matchScheduler.schedule(new MatchTime(7, MatchTime.Mode.TELEOP, MatchTime.Type.FROM_END), () -> {
            System.out.println("rumbling");
            final var rumble = robotInput.getDriverRumble();
            if(rumble.isConnected()){
                rumble.rumbleTime(150, .6);
            }
        });
        System.out.println("Scheduled some stuff for end of teleop!");
    }

    @Override
    public void autonomousInit() {
        actionChooser.setNextAction(
                new Actions.ActionQueueBuilder(
                        autonomousChooserState.createAutonomousAction(orientationSystem.getOrientation().getOrientation()),
                        teleopAction
                ) .immediatelyDoNextWhenDone(true) .canBeDone(false) .canRecycle(false) .build()
        );
        final Perspective autoPerspective = autonomousPerspectiveChooser.getSelected();
        if(autoPerspective == dimensions.getHatchManipulatorPerspective()){
            taskSystem.setCurrentTask(TaskSystem.Task.HATCH);
        } else if(autoPerspective == dimensions.getCargoManipulatorPerspective()){
            taskSystem.setCurrentTask(TaskSystem.Task.CARGO);
        }
        swerveDriveAction.setPerspective(autoPerspective);
        soundMap.getAutonomousEnable().play();
    }
    @Override
    public void autonomousPeriodic() {
        if(!teleopAction.isActive()){
            if(robotInput.getAutonomousCancelButton().isDown()){
                actionChooser.setNextAction(teleopAction);
                System.out.println("Letting teleop take over now");
            }
        }
    }

    @Override
    public void testInit() {
        dashboardMap.getLiveWindow().setEnabled(true);
    }
    // endregion

    public Clock getClock() { return clock; }
    public FrcLogger getLogger(){ return logger; }

    public SwerveDrive getDrive(){ return drive; }
    public Orientation getOrientation(){
        return orientationSystem.getOrientation();
    }
    public DistanceAccumulator getAbsoluteDistanceAccumulator(){
        return absoluteDistanceAccumulator;
    }


    public RobotDimensions getDimensions() {
        return dimensions;
    }

    public SoundMap getSoundMap(){ return soundMap; }

    public Lift getLift(){ return lift; }
    public CargoIntake getCargoIntake(){ return cargoIntake; }
    public HatchIntake getHatchIntake(){ return hatchIntake; }
    public Climber getClimber(){ return climber; }
    public TaskSystem getTaskSystem(){ return taskSystem; }

    public SurroundingProvider getSurroundingProvider() {
        return surroundingProvider;
    }
}
