package com.first1444.frc.robot2019.autonomous.actions;

import com.first1444.frc.robot2019.subsystems.Lift;
import me.retrodaredevil.action.SimpleAction;

import java.util.Objects;
import java.util.function.Supplier;

public class RaiseLift extends SimpleAction {
    private final Supplier<Lift> liftSupplier;
    private final Double position;
    private final Lift.Position liftPosition;
    public RaiseLift(Supplier<Lift> liftSupplier, double position) {
        super(true);
        this.liftSupplier = liftSupplier;
        this.position = position;
        this.liftPosition = null;
    }
    public RaiseLift(Supplier<Lift> liftSupplier, Lift.Position liftPosition) {
        super(true);
        this.liftSupplier = Objects.requireNonNull(liftSupplier);
        this.position = null;
        this.liftPosition = Objects.requireNonNull(liftPosition);
    }

    @Override
    protected void onUpdate() {
        super.onUpdate();
        final Lift lift = liftSupplier.get();
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
