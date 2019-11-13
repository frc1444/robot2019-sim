package com.first1444.frc.robot2019.autonomous.actions;

import com.first1444.frc.robot2019.actions.TimedAction;

import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * A {@link TimedAction} designed for autonomous that allows you to specify a time to wait while you are able to
 * keep waiting by having a {@link BooleanSupplier} that gives true. You can also end this action early if the other {@link BooleanSupplier}
 * returns true
 */
public class AutonomousInputWaitAction extends TimedAction {
	private final BooleanSupplier shouldWait;
	private final BooleanSupplier shouldStart;
	/**
	 * @param lastMillis The amount of time in millis for this to last
	 */
	public AutonomousInputWaitAction(long lastMillis, BooleanSupplier shouldWait, BooleanSupplier shouldStart) {
		super(true, lastMillis);
		this.shouldWait = Objects.requireNonNull(shouldWait);
		this.shouldStart = Objects.requireNonNull(shouldStart);
	}
	
	@Override
	protected void onIsDoneRequest() {
		super.onIsDoneRequest();
		if(shouldStart.getAsBoolean()){
			setDone(true);
		}
		if(shouldWait.getAsBoolean()){
			setDone(false);
		}
	}
}
