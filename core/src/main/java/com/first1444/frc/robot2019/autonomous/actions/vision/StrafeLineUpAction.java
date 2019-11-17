package com.first1444.frc.robot2019.autonomous.actions.vision;

import com.first1444.frc.robot2019.autonomous.actions.DistanceAwayLinkedAction;
import com.first1444.frc.robot2019.event.EventSender;
import com.first1444.frc.robot2019.event.SoundEvents;
import com.first1444.sim.api.Clock;
import com.first1444.sim.api.drivetrain.StrafeDrive;
import com.first1444.sim.api.selections.Selector;
import com.first1444.sim.api.sensors.Orientation;
import com.first1444.sim.api.surroundings.Surrounding;
import com.first1444.sim.api.surroundings.SurroundingProvider;
import me.retrodaredevil.action.Action;
import me.retrodaredevil.action.SimpleAction;

public class StrafeLineUpAction extends SimpleAction implements DistanceAwayLinkedAction {
	public static final double MAX_SPEED = .3;
	private static final double FAIL_NOTIFY_TIME = .1;
	private static final double MAX_FAIL_TIME = 2;
	private static final double MAX_MOVE_X = 5;

	private final Clock clock;
	private final SurroundingProvider surroundingProvider;
	private final Selector<Surrounding> surroundingSelector;
	private final StrafeDrive drive;
	private final Orientation orientation; // We won't need this until we start estimating after we lose a target
    private final double desiredSurroundingRotationDegrees;
	
	private final Action successAction;
	private final EventSender eventSender;


	/** Used for sending sounds. Set to true once we found it. Set to false when we lose it.*/
	private boolean hasFound = false;
	private Action nextAction;
	private Double failureStartTime = null;
	private Double lastFailSound = null;
	
	private double distanceAway = Double.MAX_VALUE;
	
	StrafeLineUpAction(
			Clock clock,
			SurroundingProvider surroundingProvider,
			Selector<Surrounding> surroundingSelector,
			StrafeDrive drive, Orientation orientation,
			double desiredSurroundingRotationDegrees,
			Action failAction, Action successAction, EventSender eventSender
	) {
		super(false);
		this.clock = clock;
		this.surroundingProvider = surroundingProvider;
		this.surroundingSelector = surroundingSelector;
		this.drive = drive;
		this.orientation = orientation;
		this.desiredSurroundingRotationDegrees = desiredSurroundingRotationDegrees;

		this.successAction = successAction;
		this.eventSender = eventSender;
		
		nextAction = failAction;
	}
	
	@Override
	protected void onUpdate() {
		super.onUpdate();
		final double now = clock.getTimeSeconds();
		final Surrounding surrounding = surroundingSelector.select(surroundingProvider.getSurroundings());
		final boolean failed;
		if(surrounding != null){
			if(!hasFound){
				if(eventSender != null) {
					eventSender.sendEvent(SoundEvents.TARGET_FOUND);
				}
				hasFound = true;
			}
			failed = false;
			useSurrounding(surrounding);
		} else {
			failed = true;
		}
		if(failed){
		    // TODO cache old successful surroundings and try to estimate where we are and where we need to be
			if(failureStartTime == null){
				failureStartTime = now;
			}
			if(failureStartTime + FAIL_NOTIFY_TIME < now && (lastFailSound == null || lastFailSound + 1 < now)){
				if(eventSender != null) {
					eventSender.sendEvent(SoundEvents.TARGET_FAILED);
				}
				lastFailSound = clock.getTimeSeconds();
			}
			if(failureStartTime + MAX_FAIL_TIME < System.currentTimeMillis()){
				System.out.println("Failed vision. setDone(true) now");
				setDone(true);
			}
		} else {
			failureStartTime = null;
		}
	}
	private void useSurrounding(Surrounding surrounding){
		// TODO implement this and add member field for desired orientation relative to surrounding
	}

	@Override
	public Action getNextAction() {
		return nextAction;
	}
	
	@Override
	public double getDistanceAway() {
		return distanceAway;
	}
}
