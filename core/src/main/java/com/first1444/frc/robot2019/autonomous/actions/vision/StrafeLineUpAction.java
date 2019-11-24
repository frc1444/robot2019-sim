package com.first1444.frc.robot2019.autonomous.actions.vision;

import com.first1444.frc.robot2019.autonomous.actions.DistanceAwayLinkedAction;
import com.first1444.frc.robot2019.event.EventSender;
import com.first1444.frc.robot2019.event.SoundEvents;
import com.first1444.sim.api.Clock;
import com.first1444.sim.api.Transform;
import com.first1444.sim.api.Vector2;
import com.first1444.sim.api.drivetrain.StrafeDrive;
import com.first1444.sim.api.selections.Selector;
import com.first1444.sim.api.sensors.Orientation;
import com.first1444.sim.api.surroundings.Surrounding;
import com.first1444.sim.api.surroundings.SurroundingProvider;
import me.retrodaredevil.action.Action;
import me.retrodaredevil.action.SimpleAction;

import static com.first1444.sim.api.MathUtil.minChange;
import static com.first1444.sim.api.MeasureUtil.inchesToMeters;
import static com.first1444.sim.api.Transform.transformRadians;
import static java.lang.Math.max;
import static java.lang.Math.min;

public class StrafeLineUpAction extends SimpleAction implements DistanceAwayLinkedAction {
	private static final double FAIL_NOTIFY_TIME = .1;
	private static final double MAX_FAIL_TIME = 2;
	private static final double TARGET_VALIDITY_DURATION = .5;

	private final Clock clock;
	private final SurroundingProvider surroundingProvider;
	private final Selector<Surrounding> surroundingSelector;
	private final StrafeDrive drive;
	private final Orientation orientation; // We won't need this until we start estimating after we lose a target
    private final double desiredSurroundingRotationDegrees;
    private final double targetDistanceBack;
	
	private final Action successAction;
	private final EventSender eventSender;


	/** Used for sending sounds. Set to true once we found it. Set to false when we lose it.*/
	private boolean hasFound = false;
	private Action nextAction;
	private Double failureStartTime = null;
	private Double lastFailSound = null;
	private Double successStart = null;
	
	private double distanceAway = Double.MAX_VALUE;
	
	StrafeLineUpAction(
			Clock clock,
			SurroundingProvider surroundingProvider,
			Selector<Surrounding> surroundingSelector,
			StrafeDrive drive, Orientation orientation,
			double desiredSurroundingRotationDegrees,
			double targetDistanceBack,
			Action failAction, Action successAction, EventSender eventSender
	) {
		super(false);
		this.clock = clock;
		this.surroundingProvider = surroundingProvider;
		this.surroundingSelector = surroundingSelector;
		this.drive = drive;
		this.orientation = orientation;
		this.desiredSurroundingRotationDegrees = desiredSurroundingRotationDegrees;
		this.targetDistanceBack = targetDistanceBack;

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
		if(surrounding != null && surrounding.getTimestamp() + TARGET_VALIDITY_DURATION >= now){
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
		Transform vision = surrounding.getTransform();
		vision = transformRadians(vision.getPosition().getNormalized().times(vision.getPosition().getMagnitude() - targetDistanceBack), vision.getRotationRadians());
		double yawDegrees = vision.getRotationDegrees() - desiredSurroundingRotationDegrees;
		yawDegrees = minChange(yawDegrees, 0, 360);
		double yawTurnAmount = max(-1, min(1, yawDegrees / -30));

		Transform reversed = vision.getReversed();
//		Vector2 translate = vision.getPosition().getNormalized();
        Vector2 translate = transformRadians(reversed.getPosition().times(1, 3), reversed.getRotationRadians())
				.getReversed().getPosition();

        if(translate.getMagnitude() > .1){
        	translate = translate.getNormalized();
		} else {
        	translate = translate.getNormalized().times(translate.getMagnitude() / .1);
		}

		double distanceAway = vision.getPosition().getMagnitude();
		this.distanceAway = distanceAway;
		if(distanceAway < inchesToMeters(3)){
		    nextAction = successAction;
			setDone(true);
		} else {
			drive.setControl(translate, yawTurnAmount, .6);
		}
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
