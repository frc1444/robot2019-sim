package com.first1444.frc.robot2019.autonomous.actions;

import com.first1444.frc.robot2019.actions.TimedAction;
import com.first1444.frc.robot2019.subsystems.HatchIntake;
import com.first1444.sim.api.Clock;
import me.retrodaredevil.action.Action;

import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;

import static java.util.Objects.requireNonNull;

public class HatchIntakeAction extends TimedAction {
    private final HatchIntake hatchIntake;
    private final Consumer<HatchIntake> hatchIntakeAction;
    private HatchIntakeAction(Clock clock, HatchIntake hatchIntake, Consumer<HatchIntake> hatchIntakeAction) {
        super(true, clock, .5);
        this.hatchIntake = requireNonNull(hatchIntake);
        this.hatchIntakeAction = requireNonNull(hatchIntakeAction);
    }
    public static Action createGrab(Clock clock, HatchIntake hatchIntake){
        return new HatchIntakeAction(clock, hatchIntake, HatchIntake::hold);
    }
    public static Action createDrop(Clock clock, HatchIntake hatchIntake){
        return new HatchIntakeAction(clock, hatchIntake, HatchIntake::drop);
    }

    @Override
    protected void onStart() {
        super.onStart();
        hatchIntakeAction.accept(hatchIntake);
    }
}
