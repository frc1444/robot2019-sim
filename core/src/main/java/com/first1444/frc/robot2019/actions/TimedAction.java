package com.first1444.frc.robot2019.actions;

import me.retrodaredevil.action.SimpleAction;

public class TimedAction extends SimpleAction {

	private final long lastMillis;

	private long startMillis;

	/**
	 *
	 * @param canRecycle Can this action be recycled
	 * @param lastMillis The amount of time in millis for this to last
	 */
	public TimedAction(boolean canRecycle, long lastMillis) {
		super(canRecycle);
		this.lastMillis = lastMillis;
	}
	protected long getTimeMillis(){
		return System.currentTimeMillis();
	}
	protected final long getTimeLeft(){
		return (lastMillis + startMillis) - getTimeMillis();
	}

	@Override
	protected void onStart() {
		super.onStart();
		startMillis = getTimeMillis();
	}

	@Override
	protected void onUpdate() {
		super.onUpdate();
		setDone(getTimeLeft() <= 0);
	}

}
