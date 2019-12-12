package com.first1444.frc.robot2019.autonomous.creator;

import com.first1444.frc.robot2019.Robot;
import com.first1444.frc.robot2019.autonomous.actions.vision.LineUpCreator;
import com.first1444.frc.robot2019.deepspace.SlotLevel;
import com.first1444.frc.robot2019.subsystems.Lift;
import me.retrodaredevil.action.Action;
import me.retrodaredevil.action.Actions;
import me.retrodaredevil.action.SimpleAction;
import me.retrodaredevil.action.WhenDone;

import java.util.Map;

import static com.first1444.sim.api.MeasureUtil.inchesToMeters;

public class RobotVisionPlacementCreator implements VisionPlacementCreator {
    /** NOTE: This map does not contain {@link Lift.Position#CARGO_CARGO_SHIP} */
    public static final Map<SlotLevel, Lift.Position> SLOT_MAP = Map.of(
        SlotLevel.LEVEL1, Lift.Position.LEVEL1, // although SlotLevel.LEVEL1 may refer to the CARGO_CARGO_SHIP height, when we use this map, it is not for that
        SlotLevel.LEVEL2, Lift.Position.LEVEL2,
        SlotLevel.LEVEL3, Lift.Position.LEVEL3
    );
    private final Robot robot;
    private final OperatorActionCreator operatorActionCreator;

    public RobotVisionPlacementCreator(Robot robot) {
        this.robot = robot;
        operatorActionCreator = new RobotOperatorActionCreator(robot);
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
    @SuppressWarnings("DuplicatedCode")
    private Action createLineUpWithRunner(boolean hatch, Lift.Position liftPosition, Action failAction, Action successAction){
        final boolean[] success = {false};
        final boolean[] fail = {false};

        final var lineUp = LineUpCreator.createLinkedLineUpAction(
            robot.getClock(),
            robot.getSurroundingProvider(),
            robot.getDrive(), robot.getOrientation(),
            hatch ? robot.getDimensions().getHatchManipulatorPerspective().getOffset() : robot.getDimensions().getCargoManipulatorPerspective().getOffset(),
            Actions.createRunOnce(() -> fail[0] = true),
            Actions.createRunOnce(() -> success[0] = true),
            robot.getSoundMap()
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
                        distanceLeft = lineUp.getDistanceAway();
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
                    if(distanceLeft < inchesToMeters(40)){
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
                    final Action releaseAction = hatch ? operatorActionCreator.createDropHatch() : operatorActionCreator.createReleaseCargo();
                    action = Actions.createLinkedAction(releaseAction, successAction);
                } else {
                    action = failAction;
                }
                return Actions.createLinkedActionRunner(action, WhenDone.CLEAR_ACTIVE_AND_BE_DONE, true);
            })
        ).immediatelyDoNextWhenDone(true).build();
    }
}
