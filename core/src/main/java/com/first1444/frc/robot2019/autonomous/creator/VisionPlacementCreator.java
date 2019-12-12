package com.first1444.frc.robot2019.autonomous.creator;

import com.first1444.frc.robot2019.deepspace.SlotLevel;
import me.retrodaredevil.action.Action;

public interface VisionPlacementCreator {
    Action createCargoShipPlaceHatchUseVision(Action failAction, Action successAction);
    Action createCargoShipPlaceCargoUseVision(Action failAction, Action successAction);

    Action createRocketPlaceHatchUseVision(SlotLevel slotLevel, Action failAction, Action successAction);
    Action createRocketPlaceCargoUseVision(SlotLevel slotLevel, Action failAction, Action successAction);
}
