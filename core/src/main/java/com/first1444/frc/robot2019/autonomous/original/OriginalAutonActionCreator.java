package com.first1444.frc.robot2019.autonomous.original;

import com.first1444.frc.robot2019.autonomous.creator.action.OperatorActionCreator;
import com.first1444.frc.robot2019.autonomous.creator.action.VisionPlacementCreator;
import com.first1444.frc.util.autonomous.creator.LogActionCreator;
import com.first1444.frc.util.autonomous.creator.OriginalSwerveDriveActionCreator;
import me.retrodaredevil.action.Action;
import me.retrodaredevil.action.LinkedAction;
import me.retrodaredevil.action.WhenDone;

/**
 * NOTE: Each returned action will probably not be a {@link LinkedAction} so you do not have to
 * use {@link me.retrodaredevil.action.Actions#createLinkedActionRunner(Action, WhenDone, boolean)} to wrap them.
 * Doing so is not recommended and most if not all of the time, it will do nothing.
 * <p>
 * NOTE: For each failAction and successAction, they are allowed to be null and if they're an instanceof {@link LinkedAction},
 * they will be ran like a {@link LinkedAction}
 */
@Deprecated
public interface OriginalAutonActionCreator {
    LogActionCreator getLogCreator();
    OriginalSwerveDriveActionCreator getDriveCreator();
    OperatorActionCreator getOperatorCreator();
    VisionPlacementCreator getVisionCreator();
}
