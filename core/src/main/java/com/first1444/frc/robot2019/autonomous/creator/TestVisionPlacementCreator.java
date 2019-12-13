package com.first1444.frc.robot2019.autonomous.creator;

import com.first1444.frc.robot2019.deepspace.SlotLevel;
import com.first1444.frc.util.autonomous.creator.LogActionCreator;
import me.retrodaredevil.action.Action;
import me.retrodaredevil.action.Actions;
import me.retrodaredevil.action.WhenDone;

public class TestVisionPlacementCreator implements VisionPlacementCreator {
    private final LogActionCreator logActionCreator;

    public TestVisionPlacementCreator(LogActionCreator logActionCreator) {
        this.logActionCreator = logActionCreator;
    }

    @Override
    public Action createCargoShipPlaceHatchUseVision(Action failAction, Action successAction) {
        return Actions.createLinkedActionRunner(
            Actions.createLinkedAction(logActionCreator.createLogMessageAction("Placing hatch at cargo ship using vision"), successAction),
            WhenDone.CLEAR_ACTIVE_AND_BE_DONE, true
        );
    }

    @Override
    public Action createCargoShipPlaceCargoUseVision(Action failAction, Action successAction) {
        return Actions.createLinkedActionRunner(
            Actions.createLinkedAction(logActionCreator.createLogMessageAction("Placing cargo at cargo ship using vision"), successAction),
            WhenDone.CLEAR_ACTIVE_AND_BE_DONE, true
        );
    }

    @Override
    public Action createRocketPlaceCargoUseVision(SlotLevel slotLevel, Action failAction, Action successAction) {
        return Actions.createLinkedActionRunner(
            Actions.createLinkedAction(logActionCreator.createLogMessageAction("Placing cargo on rocket at " + slotLevel + " using vision"), successAction),
            WhenDone.CLEAR_ACTIVE_AND_BE_DONE, true
        );
    }

    @Override
    public Action createRocketPlaceHatchUseVision(SlotLevel slotLevel, Action failAction, Action successAction) {
        return Actions.createLinkedActionRunner(
            Actions.createLinkedAction(logActionCreator.createLogMessageAction("Placing hatch on rocket at " + slotLevel + " using vision"), successAction),
            WhenDone.CLEAR_ACTIVE_AND_BE_DONE, true
        );
    }
}
