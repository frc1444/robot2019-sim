/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.												*/
/* Open Source Software - may be modified and shared by FRC teams. The code	 */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.																															 */
/*----------------------------------------------------------------------------*/

package com.first1444.frc.robot2019;

import com.first1444.frc.robot2019.actions.OperatorAction;
import com.first1444.frc.robot2019.actions.SwerveCalibrateAction;
import com.first1444.frc.robot2019.actions.SwerveDriveAction;
import com.first1444.frc.robot2019.autonomous.AutonomousChooserState;
import com.first1444.frc.robot2019.autonomous.AutonomousModeCreator;
import com.first1444.frc.robot2019.autonomous.RobotAutonActionCreator;
import com.first1444.frc.robot2019.autonomous.actions.TimedCargoIntake;
import com.first1444.frc.robot2019.autonomous.actions.vision.LineUpCreator;
import com.first1444.frc.robot2019.event.EventSender;
import com.first1444.frc.robot2019.event.SoundEvents;
import com.first1444.frc.robot2019.event.TCPEventSender;
import com.first1444.frc.robot2019.input.DefaultRobotInput;
import com.first1444.frc.robot2019.input.InputUtil;
import com.first1444.frc.robot2019.input.RobotInput;
import com.first1444.frc.robot2019.subsystems.*;
import com.first1444.frc.robot2019.subsystems.implementations.*;
import com.first1444.frc.robot2019.vision.BestVisionPacketSelector;
import com.first1444.frc.robot2019.vision.DefaultVisionPacketProvider;
import com.first1444.frc.robot2019.vision.PacketListener;
import com.first1444.frc.robot2019.vision.VisionSupplier;
import com.first1444.frc.util.DynamicSendableChooser;
import com.first1444.frc.util.OrientationSendable;
import com.first1444.sim.api.Clock;
import com.first1444.sim.api.drivetrain.swerve.FourWheelSwerveDrive;
import com.first1444.sim.api.drivetrain.swerve.FourWheelSwerveDriveData;
import com.first1444.sim.api.drivetrain.swerve.SwerveDrive;
import com.first1444.sim.api.frc.FrcDriverStation;
import com.first1444.sim.api.frc.IterativeRobotAdapter;
import com.first1444.sim.api.scheduler.match.DefaultMatchScheduler;
import com.first1444.sim.api.scheduler.match.MatchScheduler;
import com.first1444.sim.api.scheduler.match.MatchTime;
import com.first1444.sim.api.sensors.Orientation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import me.retrodaredevil.action.*;
import me.retrodaredevil.controller.implementations.ControllerPartCreator;
import me.retrodaredevil.controller.MutableControlConfig;
import me.retrodaredevil.controller.ControlConfig;
import me.retrodaredevil.controller.PartUpdater;
import me.retrodaredevil.controller.output.ControllerRumble;
import me.retrodaredevil.controller.types.StandardControllerInput;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobotAdapter {

	private final FrcDriverStation driverStation;
	
	private final OrientationSystem orientationSystem;
	private final RobotDimensions dimensions;

	private final PartUpdater partUpdater = new PartUpdater();
	private final ControlConfig controlConfig;
	{
		MutableControlConfig controlConfig = new MutableControlConfig();
		// *edit values of controlConfig if desired*
		controlConfig.switchToSquareInputThreshold = 1.2;
		controlConfig.fullAnalogDeadzone = .075;
		controlConfig.analogDeadzone = .02;
		controlConfig.cacheAngleAndMagnitudeInUpdate = false;
		this.controlConfig = controlConfig;
	}
	private final RobotInput robotInput;

	private final DynamicSendableChooser<Perspective> autonomousPerspectiveChooser;
	private final SwerveDrive drive;
	private final CargoIntake cargoIntake;
	private final Climber climber;
	private final HatchIntake hatchIntake;
	private final Lift lift;
	private final TaskSystem taskSystem;
	private final MatchScheduler matchScheduler;
	
	private final PacketListener packetListener;
	private final EventSender soundSender;

	/** An {@link Action} that updates certain subsystems only when the robot is enabled*/
	private final ActionMultiplexer enabledSubsystemUpdater;
	/** An {@link Action} that updates certain subsystems all the time. If {@link #enabledSubsystemUpdater} is updated, this is updated after that*/
	private final ActionMultiplexer constantSubsystemUpdater;
	/** The {@link ActionChooser} that handles an action that updates subsystems*/
	private final ActionChooser actionChooser;

	private final Action teleopAction;
	private final SwerveDriveAction swerveDriveAction;
//	private final Action testAction;
	private final AutonomousChooserState autonomousChooserState;
	
	private enum RobotMode {TELEOP, TEST, AUTO, DISABLED}
	private RobotMode lastMode = RobotMode.DISABLED;
	
	
	// region Initialize
	/** Used to initialize final fields.*/
	public Robot(
			FrcDriverStation driverStation,
			Clock clock,
			ShuffleboardMap shuffleboardMap,
			StandardControllerInput controller, ControllerPartCreator port1, ControllerPartCreator port2, ControllerRumble rumble,
			Orientation orientation,
			FourWheelSwerveDriveData fourWheelSwerveData,
			Lift lift, CargoIntake cargoIntake, HatchIntake hatchIntake, Climber climber,
			Action extraAction
	){
	    this.driverStation = driverStation;
		robotInput = new DefaultRobotInput(
				controller,
				InputUtil.createJoystick(port1),
				InputUtil.createAttackJoystick(port2),
				rumble
		);
		partUpdater.addPartAssertNotPresent(robotInput);
		partUpdater.updateParts(controlConfig); // update this so when calling get methods don't throw exceptions

		orientationSystem = new OrientationSystem(shuffleboardMap, orientation, robotInput);
//		final Gyro gyro = new DummyGyro(0);
		
		final SendableChooser<Boolean> cameraIDSwitchedChooser = new SendableChooser<>();
		cameraIDSwitchedChooser.setDefaultOption("NOT FLIPPED", false);
		cameraIDSwitchedChooser.setDefaultOption("FLIPPED", true);
		shuffleboardMap.getDevTab().add("Camera IDs Flipped", cameraIDSwitchedChooser);
		dimensions = new DynamicRobotDimensions(Constants.Dimensions.INSTANCE, cameraIDSwitchedChooser::getSelected);


		OrientationSendable.addOrientation(shuffleboardMap.getUserTab(), this::getOrientation);
		
		autonomousPerspectiveChooser = new DynamicSendableChooser<>();
		autonomousPerspectiveChooser.setDefaultOption("Auto Cam", null);
		autonomousPerspectiveChooser.addOption("Hatch Cam", dimensions.getHatchManipulatorPerspective());
		autonomousPerspectiveChooser.addOption("Cargo Cam", dimensions.getCargoManipulatorPerspective());
		autonomousPerspectiveChooser.addOption("Driver Station (blind field centric)", Perspective.DRIVER_STATION);
		autonomousPerspectiveChooser.addOption("Jumbotron on Right", Perspective.JUMBOTRON_ON_RIGHT);
		autonomousPerspectiveChooser.addOption("Jumbotron on Left", Perspective.JUMBOTRON_ON_LEFT);
		shuffleboardMap.getUserTab().add("Autonomous Perspective", autonomousPerspectiveChooser).withSize(2, 1).withPosition(9, 4);

		final FourWheelSwerveDrive drive = new FourWheelSwerveDrive(fourWheelSwerveData);

		final var taskSystem = new DefaultTaskSystem(robotInput);
		final var matchScheduler = new DefaultMatchScheduler(driverStation, clock);
		this.drive = drive;
		this.cargoIntake = cargoIntake;
		this.climber = climber;
		this.hatchIntake = hatchIntake;
		this.lift = lift;
		this.taskSystem = taskSystem;
		this.matchScheduler = matchScheduler;
		
		packetListener = new PacketListener(5801);
		soundSender = new TCPEventSender(5809);
		
		enabledSubsystemUpdater = new Actions.ActionMultiplexerBuilder(
				Actions.createRunForeverRecyclable(drive), lift, cargoIntake, climber, hatchIntake
		).clearAllOnEnd(false).canRecycle(true).build();
		
		constantSubsystemUpdater = new Actions.ActionMultiplexerBuilder( // NOTE, without forceUpdateInOrder(true), these will not update in order
				orientationSystem,
				taskSystem,
				Actions.createRunForever(matchScheduler),
				new SwerveCalibrateAction(this::getDrive, robotInput),
				extraAction
		).clearAllOnEnd(false).canRecycle(false).build();
		actionChooser = Actions.createActionChooser(WhenDone.CLEAR_ACTIVE);

		swerveDriveAction = new SwerveDriveAction(this::getDrive, this::getOrientation, this::getTaskSystem, robotInput, getVisionSupplier(), getDimensions());
		teleopAction = new Actions.ActionMultiplexerBuilder(
				swerveDriveAction,
				new OperatorAction(this, robotInput)
		).clearAllOnEnd(false).canBeDone(false).canRecycle(true).build();
//		testAction = new TestAction(robotInput);
		autonomousChooserState = new AutonomousChooserState(
				shuffleboardMap,  // this will add stuff to the dashboard
				new AutonomousModeCreator(new RobotAutonActionCreator(this), dimensions),
				robotInput
		);

		System.out.println("Finished constructor");
		packetListener.start();
//		shuffleboardMap.getUserTab().add("Camera", CameraServer.getInstance().startAutomaticCapture());

		System.out.println("Finished robot init!");
	}
	
//	@Override
	public void close() { // TODO close stuff!
//		super.close();
		packetListener.interrupt();
		System.out.println("close() method called! Robot program must be ending!");
	}
	

	// endregion
	
	// region Overridden Methods
	
	/** Called before every other period method no matter what state the robot is in*/
	@Override
	public void robotPeriodic() {
	    partUpdater.updateParts(controlConfig);

		actionChooser.update(); // update Actions that control the subsystems
		
		if(driverStation.isEnabled()){
			enabledSubsystemUpdater.update(); // update subsystems when robot is enabled
		}
		constantSubsystemUpdater.update(); // update subsystems that are always updated
	}
	
	/** Called when robot is disabled and in between switching between modes such as teleop and autonomous*/
	@Override
	public void disabledInit() {
		actionChooser.setToClearAction();
		if(enabledSubsystemUpdater.isActive()) {
			enabledSubsystemUpdater.end();
		}
		if(lastMode == RobotMode.TELEOP){
			soundSender.sendEvent(SoundEvents.MATCH_END);
		} else {
			soundSender.sendEvent(SoundEvents.DISABLE);
		}
		lastMode = RobotMode.DISABLED;
	}

	/** Called when going into teleop mode */
	@Override
	public void teleopInit() {
		actionChooser.setNextAction(new Actions.ActionMultiplexerBuilder(
				teleopAction
		).canRecycle(false).canBeDone(true).build());
		swerveDriveAction.setPerspective(Perspective.DRIVER_STATION);
		soundSender.sendEvent(SoundEvents.TELEOP_ENABLE);
		matchScheduler.schedule(new MatchTime(1.2, MatchTime.Mode.TELEOP, MatchTime.Type.FROM_END), () -> {
			System.out.println("Stowing cargo intake"); // good
			cargoIntake.stow();
		});
		matchScheduler.schedule(new MatchTime(.5, MatchTime.Mode.TELEOP, MatchTime.Type.FROM_END), () -> {
			new TimedCargoIntake(400, this::getCargoIntake, 1); // TODO add this to something
		}); // good
		matchScheduler.schedule(new MatchTime(.5, MatchTime.Mode.TELEOP, MatchTime.Type.FROM_END), () -> {
			System.out.println("Dropping hatch"); // good
			hatchIntake.drop();
		});
		matchScheduler.schedule(new MatchTime(7, MatchTime.Mode.TELEOP, MatchTime.Type.FROM_END), () -> {
			System.out.println("rumbling"); // good
			final var rumble = robotInput.getDriverRumble();
			if(rumble.isConnected()){
				rumble.rumbleTime(150, .6);
			}
		});
		System.out.println("Scheduled some stuff for end of teleop!");
		lastMode = RobotMode.TELEOP;
	}

	/** Called first thing when match starts. Autonomous is active for 15 seconds*/
	@Override
	public void autonomousInit() {
		actionChooser.setNextAction(
				new Actions.ActionQueueBuilder(
						autonomousChooserState.createAutonomousAction(orientationSystem.getStartingOrientation()),
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
		soundSender.sendEvent(SoundEvents.AUTONOMOUS_ENABLE);
		lastMode = RobotMode.AUTO;
	}
	/** Called constantly during autonomous*/
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
//		actionChooser.setNextAction(testAction);
		actionChooser.setNextAction(new Actions.ActionQueueBuilder(
//				new TurnToOrientation(-90, this::getDrive, this::getOrientation),
//				new GoStraight(10, .2, 0, 1, 90.0, this::getDrive, this::getOrientation),
				LineUpCreator.createLineUpAction(
						new DefaultVisionPacketProvider(
								dimensions.getHatchManipulatorPerspective(),
								getVisionSupplier(),
								dimensions.getHatchCameraID(),
								new BestVisionPacketSelector(),
								Constants.VISION_PACKET_VALIDITY_TIME
						),
						this::getDrive, this::getOrientation,
						Actions.createRunOnce(() -> System.out.println("Failed!")), Actions.createRunOnce(() -> System.out.println("Success!")),
						getSoundSender()
				),
				Actions.createRunOnce(() -> robotInput.getDriverRumble().rumbleTime(500, .2))
		).canRecycle(false).canBeDone(true).immediatelyDoNextWhenDone(true).build());
		lastMode = RobotMode.TEST;
	}
	// endregion
	
	public SwerveDrive getDrive(){ return drive; }
	public Orientation getOrientation(){
		return orientationSystem.getOrientation();
	}
	
	public VisionSupplier getVisionSupplier() {
		return packetListener;
	}
	
	public RobotDimensions getDimensions() {
		return dimensions;
	}
	
	public EventSender getSoundSender(){
		return soundSender;
	}
	
	public Lift getLift(){ return lift; }
	public CargoIntake getCargoIntake(){ return cargoIntake; }
	public HatchIntake getHatchIntake(){ return hatchIntake; }
	public Climber getClimber(){ return climber; }
	public TaskSystem getTaskSystem(){ return taskSystem; }
}
