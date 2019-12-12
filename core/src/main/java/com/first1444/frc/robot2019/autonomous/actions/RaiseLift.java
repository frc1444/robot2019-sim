package com.first1444.frc.robot2019.autonomous.actions;

import com.first1444.frc.robot2019.subsystems.Lift;
import me.retrodaredevil.action.SimpleAction;

import static java.util.Objects.requireNonNull;

public class RaiseLift extends SimpleAction {
    private final Lift lift;
    private final Double position;
    private final Lift.Position liftPosition;
    public RaiseLift(Lift lift, double position) {
        super(true);
        this.lift = requireNonNull(lift);
        this.position = position;
        this.liftPosition = null;
    }
    public RaiseLift(Lift lift, Lift.Position liftPosition) {
        super(true);
        this.lift = requireNonNull(lift);
        this.position = null;
        this.liftPosition = requireNonNull(liftPosition);
    }

    @Override
    protected void onUpdate() {
        super.onUpdate();
        if(position != null){
            lift.setDesiredPosition(position);
        } else if(liftPosition != null){
            lift.setDesiredPosition(liftPosition);
        } else {
            throw new AssertionError();
        }
        setDone(lift.isDesiredPositionReached());
    }
}
