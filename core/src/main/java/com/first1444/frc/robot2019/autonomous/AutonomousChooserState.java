package com.first1444.frc.robot2019.autonomous;

import com.first1444.dashboard.advanced.implementations.chooser.ChooserSendable;
import com.first1444.dashboard.advanced.implementations.chooser.MutableMappedChooserProvider;
import com.first1444.dashboard.advanced.implementations.chooser.SimpleMappedChooserProvider;
import com.first1444.dashboard.shuffleboard.ComponentMetadataHelper;
import com.first1444.dashboard.shuffleboard.SendableComponent;
import com.first1444.dashboard.shuffleboard.ShuffleboardContainer;
import com.first1444.dashboard.shuffleboard.implementations.ShuffleboardLayoutComponent;
import com.first1444.frc.robot2019.Constants;
import com.first1444.frc.robot2019.DashboardMap;
import com.first1444.frc.robot2019.autonomous.actions.AutonomousInputWaitAction;
import com.first1444.frc.robot2019.autonomous.options.AfterComplete;
import com.first1444.frc.robot2019.autonomous.options.AutonomousType;
import com.first1444.frc.robot2019.autonomous.options.LineUpType;
import com.first1444.frc.robot2019.autonomous.options.StartingPosition;
import com.first1444.frc.robot2019.deepspace.GamePieceType;
import com.first1444.frc.robot2019.deepspace.SlotLevel;
import com.first1444.frc.robot2019.input.RobotInput;
import com.first1444.frc.util.valuemap.ValueMap;
import com.first1444.frc.util.valuemap.sendable.MutableValueMapSendable;
import com.first1444.sim.api.Clock;
import com.first1444.sim.api.Transform2;
import com.first1444.sim.api.Vector2;
import com.first1444.sim.api.distance.MutableDistanceAccumulator;
import me.retrodaredevil.action.Action;
import me.retrodaredevil.action.Actions;

import java.io.PrintWriter;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import static java.util.Objects.requireNonNull;

public class AutonomousChooserState {
    private final Clock clock;
    private final MutableDistanceAccumulator absoluteDistanceAccumulator;
    private final AutonomousModeCreator autonomousModeCreator;
    private final RobotInput robotInput;

    @SuppressWarnings("UnusedAssignment") // this is used in the constructor and it's initialization to false is necessary
    private boolean readyToListen = false;
    private final MutableMappedChooserProvider<AutonomousType> autonomousChooser;
    private final MutableMappedChooserProvider<StartingPosition> startingPositionChooser;
    private final MutableMappedChooserProvider<GamePieceType> gamePieceChooser;
    private final MutableMappedChooserProvider<SlotLevel> levelChooser;
    private final MutableMappedChooserProvider<LineUpType> lineUpChooser;
    private final MutableMappedChooserProvider<AfterComplete> afterCompleteChooser;
    private final ValueMap<AutonomousConfigKey> autonomousConfigKeyValueMap;

    public AutonomousChooserState(DashboardMap dashboardMap, Clock clock, MutableDistanceAccumulator absoluteDistanceAccumulator, AutonomousModeCreator autonomousModeCreator, RobotInput robotInput){
        this.clock = clock;
        this.absoluteDistanceAccumulator = absoluteDistanceAccumulator;
        this.autonomousModeCreator = autonomousModeCreator;
        this.robotInput = robotInput;
        final ShuffleboardContainer layout = dashboardMap.getUserTab()
                .add("Autonomous", ShuffleboardLayoutComponent.LIST, (metadata) -> new ComponentMetadataHelper(metadata)
                        .setSize(2, 5)
                        .setPosition(0, 0));
        autonomousChooser = new SimpleMappedChooserProvider<>(key -> onAutonomousChange());
        startingPositionChooser = new SimpleMappedChooserProvider<>(key -> onStartingPositionChange());
        gamePieceChooser = new SimpleMappedChooserProvider<>();
        levelChooser = new SimpleMappedChooserProvider<>();
        lineUpChooser = new SimpleMappedChooserProvider<>();
        afterCompleteChooser = new SimpleMappedChooserProvider<>();
        final var valueMapSendable = new MutableValueMapSendable<>(AutonomousConfigKey.class);
        layout.add("Config", new SendableComponent<>(valueMapSendable), (metadata) -> new ComponentMetadataHelper(metadata)
                .setProperties(Constants.ROBOT_PREFERENCES_PROPERTIES));
        autonomousConfigKeyValueMap = valueMapSendable.getMutableValueMap();

        addAutoOptions();
        updateStartingPositionChooser();
        updateGamePieceChooser();
        updateLevelChooser();
        updateLineUpChooser();
        updateAfterCompleteChooser();

        layout.add("Autonomous Chooser", new SendableComponent<>(new ChooserSendable(autonomousChooser)),
                (metadata) -> new ComponentMetadataHelper(metadata).setSize(2, 1).setPosition(0, 0));
        layout.add("Starting Position Chooser", new SendableComponent<>(new ChooserSendable(startingPositionChooser)),
                (metadata) -> new ComponentMetadataHelper(metadata).setSize(2, 1).setPosition(0, 1));
        layout.add("Game Piece Chooser", new SendableComponent<>(new ChooserSendable(gamePieceChooser)),
                (metadata) -> new ComponentMetadataHelper(metadata).setSize(2, 1).setPosition(0, 2));
        layout.add("Level Chooser", new SendableComponent<>(new ChooserSendable(levelChooser)),
                (metadata) -> new ComponentMetadataHelper(metadata).setSize(2, 1).setPosition(0, 3));
        layout.add("Line Up Chooser", new SendableComponent<>(new ChooserSendable(lineUpChooser)),
                (metadata) -> new ComponentMetadataHelper(metadata).setSize(2, 1).setPosition(0, 4));
        layout.add("After Complete Chooser", new SendableComponent<>(new ChooserSendable(afterCompleteChooser)),
                (metadata) -> new ComponentMetadataHelper(metadata).setSize(2, 1).setPosition(0, 5));
        readyToListen = true;
        onStartingPositionChange();
    }
    private void onAutonomousChange(){
        if(!readyToListen){
            return;
        }
        updateStartingPositionChooser();
        updateGamePieceChooser();
        updateLevelChooser();
        updateLineUpChooser();
        updateAfterCompleteChooser();
    }
    private void onStartingPositionChange(){
        if(!readyToListen){
            return;
        }
        StartingPosition position = startingPositionChooser.getSelected();
        final double x;
        if(position == null) {
            x = 0;
        } else if (position == StartingPosition.LEFT) {
            x = -1.1;
        } else if (position == StartingPosition.MIDDLE) {
            x = 0;
        } else if (position == StartingPosition.MIDDLE_LEFT) {
            x = -.2;
        } else if (position == StartingPosition.MIDDLE_RIGHT) {
            x = .2;
        } else if (position == StartingPosition.RIGHT) {
            x = 1.1;
        } else {
            throw new UnsupportedOperationException("Unknown position: " + position);
        }
        absoluteDistanceAccumulator.setPosition(new Vector2(x, -6.7));
    }
    public Action createAutonomousAction(Transform2 startingTransform){
        final AutonomousType type = autonomousChooser.getSelected();
        if(type == null){
            throw new NullPointerException("The autonomous type cannot be null!");
        }
        final StartingPosition startingPosition = startingPositionChooser.getSelected();
        final GamePieceType gamePiece = gamePieceChooser.getSelected();
        final SlotLevel slotLevel = levelChooser.getSelected();
        final LineUpType lineUpType = lineUpChooser.getSelected();
        final AfterComplete afterComplete = afterCompleteChooser.getSelected();
        try {
            return Actions.createLogAndEndTryCatchAction(
                    new Actions.ActionQueueBuilder(
                            new AutonomousInputWaitAction(
                                    clock,
                                    autonomousConfigKeyValueMap.getDouble(AutonomousConfigKey.WAIT_TIME),
                                    () -> robotInput.getAutonomousWaitButton().isDown(),
                                    () -> robotInput.getAutonomousStartButton().isDown()
                            ),
                            autonomousModeCreator.createAction(new AutonomousSettings(type, startingPosition, gamePiece, slotLevel, lineUpType, afterComplete), startingTransform)
                    ).canRecycle(false).canBeDone(true).immediatelyDoNextWhenDone(true).build(),
                    Throwable.class, new PrintWriter(System.err)
            );
        } catch (IllegalArgumentException ex){
            ex.printStackTrace();
            System.out.println("One of our choosers must not have been set correctly!");
        }
        return Actions.createRunOnce(() -> System.out.println("This is the autonomous action because there was an exception when creating the one we wanted."));
    }
    private void addAutoOptions(){
        autonomousChooser.addOption(AutonomousType.DO_NOTHING.getName(), AutonomousType.DO_NOTHING, true);
        for(AutonomousType type : AutonomousType.values()){
            if(type != AutonomousType.DO_NOTHING){
                autonomousChooser.addOption(type.getName(), type);
            }
        }
    }
    private void updateStartingPositionChooser(){
        Map<String, StartingPosition> selectionMap = new LinkedHashMap<>();
        String defaultKey = null;
        final AutonomousType type = autonomousChooser.getSelected();
        final Collection<StartingPosition> startingPositions = type.getStartingPositions();
        if(startingPositions.isEmpty()){
            selectionMap.put("Neither", null);
            defaultKey = "Neither";
        } else {
            for(StartingPosition position : startingPositions){
                selectionMap.put(position.toString(), position);
                defaultKey = position.toString();
            }
        }
        startingPositionChooser.set(selectionMap, requireNonNull(defaultKey));
    }
    private void updateGamePieceChooser(){
        Map<String, GamePieceType> selectionMap = new LinkedHashMap<>();
        String defaultKey = null;
        final AutonomousType type = autonomousChooser.getSelected();
        final Collection<GamePieceType> gamePieces = type.getGamePieces();
        if(gamePieces.isEmpty()){
            selectionMap.put("Neither", null);
            defaultKey = "Neither";
        } else {
            for(GamePieceType gamePiece : gamePieces){
                selectionMap.put(gamePiece.toString(), gamePiece);
                defaultKey = gamePiece.toString();
            }
        }
        gamePieceChooser.set(selectionMap, requireNonNull(defaultKey));
    }
    private void updateLevelChooser(){
        Map<String, SlotLevel> selectionMap = new LinkedHashMap<>();
        String defaultKey = null;
        final AutonomousType type = autonomousChooser.getSelected();
        final Collection<SlotLevel> slotLevels = type.getSlotLevels();
        if(slotLevels.isEmpty()){
            selectionMap.put("None", null);
            defaultKey = "None";
        } else {
            for(SlotLevel level : slotLevels){
                selectionMap.put(level.toString(), level);
                defaultKey = level.toString();
            }
        }
        levelChooser.set(selectionMap, requireNonNull(defaultKey));
    }
    private void updateLineUpChooser(){
        Map<String, LineUpType> selectionMap = new HashMap<>();
        String defaultKey = null;
        final AutonomousType type = autonomousChooser.getSelected();
        final Collection<LineUpType> lineUpTypes = type.getLineUpTypes();
        if(lineUpTypes.isEmpty()){
            throw new AssertionError("lineUpTypes should never be empty!");
        }
        for(LineUpType lineUpType : lineUpTypes){
            selectionMap.put(lineUpType.toString(), lineUpType);
            defaultKey = lineUpType.toString();
        }
        if(lineUpTypes.contains(LineUpType.NO_VISION)){
            defaultKey = LineUpType.NO_VISION.toString();
        }
        lineUpChooser.set(selectionMap, requireNonNull(defaultKey));
    }
    private void updateAfterCompleteChooser(){
        Map<String, AfterComplete> selectionMap = new HashMap<>();
        final AutonomousType type = autonomousChooser.getSelected();
        final Collection<AfterComplete> afterCompleteOptions = type.getAfterCompleteOptions();
        final String doNothingString = "Do nothing";
        selectionMap.put(doNothingString, null);

        for(AfterComplete afterComplete : afterCompleteOptions){
            selectionMap.put(afterComplete.toString(), afterComplete);
        }
        afterCompleteChooser.set(selectionMap, doNothingString);
    }

}
