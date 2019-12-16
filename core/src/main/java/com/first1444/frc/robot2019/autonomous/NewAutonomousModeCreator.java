package com.first1444.frc.robot2019.autonomous;

import com.first1444.frc.robot2019.autonomous.creator.AutonomousActionCreator;
import com.first1444.frc.robot2019.autonomous.options.AutonomousType;
import com.first1444.frc.robot2019.deepspace.GamePieceType;
import com.first1444.sim.api.Rotation2;
import com.first1444.sim.api.Transform2;
import com.first1444.sim.api.Vector2;
import me.retrodaredevil.action.Action;
import me.retrodaredevil.action.ActionQueue;
import me.retrodaredevil.action.Actions;
import org.jetbrains.annotations.NotNull;

import static com.first1444.sim.api.MeasureUtil.inchesToMeters;
import static java.util.Objects.requireNonNull;

public class NewAutonomousModeCreator implements AutonomousModeCreator {
    private final AutonomousActionCreator creator;

    public NewAutonomousModeCreator(AutonomousActionCreator creator) {
        this.creator = creator;
    }

    @Override
    public Action createAction(AutonomousSettings settings, Transform2 startingTransform) {
        final ActionQueue actionQueue = new Actions.ActionQueueBuilder()
                .canRecycle(false)
                .canBeDone(true)
                .immediatelyDoNextWhenDone(true) // once an action is finished, do the next one immediately
                .build();
        if(settings.getGamePieceType() == GamePieceType.HATCH){
            actionQueue.add(Actions.createRunOnce(creator.getOperatorCreator().createGrabHatch()));
        }
        AutonomousType autonomousType = requireNonNull(settings.getAutonomousType());
        if (autonomousType == AutonomousType.DO_NOTHING){
            actionQueue.add(creator.getLogCreator().createLogMessageAction("Do nothing autonomous action starting and complete!"));
        } else if(autonomousType == AutonomousType.CROSS_LINE_FORWARD){
            System.out.println("Starting at " + startingTransform);
            actionQueue.add(creator.getLogCreator().createLogMessageAction("Cross line forward autonomous starting!"));
            actionQueue.add(creator.getDriveCreator().createMoveToAbsolute(new Vector2(startingTransform.getX(), -4), .3)); // cross line
            actionQueue.add(creator.getLogCreator().createLogMessageAction("Cross line forward autonomous ending!"));
        } else {
            System.err.println("Doing nothing for autonomousType=" + autonomousType);
        }
        return actionQueue;
    }
}
