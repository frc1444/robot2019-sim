package com.first1444.frc.robot2019.subsystems.implementations;

import com.first1444.frc.robot2019.Constants;
import com.first1444.frc.robot2019.subsystems.Lift;
import com.first1444.frc.util.reportmap.ReportMap;
import me.retrodaredevil.action.SimpleAction;

import java.util.Objects;

public class DummyLift extends SimpleAction implements Lift {
	private static final String LIFT = "Lift";
	private static final String LIFT_REACHED = "Lift Position Reached";
	private static final long TIME_UNTIL_REACH = 1000;
	private final ReportMap reportMap;
	
	private LiftMode liftMode = LiftMode.SPEED;
	private double position;
	private long lastPositionChange = 0;
	private boolean positionReached = false;
	
	public DummyLift(ReportMap reportMap) {
		super(true);
		this.reportMap = reportMap;
	}
	
	@Override
	protected void onStart() {
		super.onStart();
		reportMap.report(LIFT, "robot enabled");
	}
	
	@Override
	protected void onUpdate() {
		super.onUpdate();
		final String infoString;
		if(liftMode == LiftMode.POSITION){
			if(lastPositionChange + TIME_UNTIL_REACH <= System.currentTimeMillis()){
				positionReached = true;
				infoString = "YES - Reached";
			} else {
				positionReached = false;
				infoString = "NO - Going";
			}
		} else {
			positionReached = false;
			infoString = "NO - Speed Mode";
		}
		reportMap.report(LIFT_REACHED, infoString);
	}
	
	@Override
	protected void onEnd(boolean peacefullyEnded) {
		super.onEnd(peacefullyEnded);
		reportMap.report(LIFT, "robot disabled");
		reportMap.report(LIFT_REACHED, "NO - disabled");
		liftMode = LiftMode.SPEED;
		position = 0;
		positionReached = false;
	}
	
	@Override
	public void setDesiredPosition(double desiredPosition) {
		reportMap.report(LIFT, "position " + Constants.DECIMAL_FORMAT.format(desiredPosition));
		liftMode = LiftMode.POSITION;
		if(desiredPosition != position){
			position = desiredPosition;
			onNewPosition();
		}
	}
	
	@Override
	public void setDesiredPosition(Position desiredPosition) {
		Objects.requireNonNull(desiredPosition);
		reportMap.report(LIFT, "position " + desiredPosition.toString());
		liftMode = LiftMode.POSITION;
		final double newPosition = Constants.LIFT_POSITION_MAP.get(desiredPosition);
		if(newPosition != position){
			position = newPosition;
			onNewPosition();
		}
	}
	private void onNewPosition(){
		lastPositionChange = System.currentTimeMillis();
	}
	
	@Override
	public void setManualSpeed(double speed, boolean overrideSpeedSafety) {
		reportMap.report(LIFT, "speed " + Constants.DECIMAL_FORMAT.format(speed) + " override: " + overrideSpeedSafety);
		liftMode = LiftMode.SPEED;
	}
	
	@Override
	public void lockCurrentPosition() {
		reportMap.report(LIFT, "position locked");
		liftMode = LiftMode.POSITION;
	}
	
	@Override
	public LiftMode getLiftMode() {
		return liftMode;
	}
	
	@Override
	public boolean isDesiredPositionReached() {
		return liftMode == LiftMode.POSITION && positionReached;
	}
}
