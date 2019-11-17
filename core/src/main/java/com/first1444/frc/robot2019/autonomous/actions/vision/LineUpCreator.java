package com.first1444.frc.robot2019.autonomous.actions.vision;

import com.first1444.frc.robot2019.autonomous.actions.DistanceAwayLinkedAction;
import com.first1444.frc.robot2019.event.EventSender;
import com.first1444.sim.api.Clock;
import com.first1444.sim.api.drivetrain.swerve.SwerveDrive;
import com.first1444.sim.api.sensors.Orientation;
import com.first1444.sim.api.surroundings.SurroundingProvider;
import me.retrodaredevil.action.Action;
import me.retrodaredevil.action.Actions;
import me.retrodaredevil.action.WhenDone;

import java.util.function.Supplier;

public final class LineUpCreator {
	private LineUpCreator() { throw new UnsupportedOperationException(); }
	
	public static Action createLineUpAction(
			Clock clock,
			SurroundingProvider surroundingProvider,
			SwerveDrive drive, Orientation orientation,
			Action failAction, Action successAction, EventSender eventSender){
		return Actions.createLinkedActionRunner(
				createLinkedLineUpAction(clock, surroundingProvider, drive, orientation, failAction, successAction, eventSender),
				WhenDone.CLEAR_ACTIVE_AND_BE_DONE,
				true
		);
	}
	public static DistanceAwayLinkedAction createLinkedLineUpAction(
			Clock clock,
			SurroundingProvider surroundingProvider,
			SwerveDrive drive, Orientation orientation,
			Action failAction, Action successAction, EventSender eventSender){
		// TODO Make surrounding selector not null
		// TODO make desired orientation not 0
		return new StrafeLineUpAction(clock, surroundingProvider, null, drive, orientation, 0.0, failAction, successAction, eventSender);
	}
}
