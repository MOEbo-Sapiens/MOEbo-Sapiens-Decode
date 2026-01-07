package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.states.IntakingState;
import org.firstinspires.ftc.teamcode.states.ShootingState;

import java.util.function.Supplier;

public enum StateFactory {
    INTAKING(IntakingState::new),
    SHOOTING(ShootingState::new);

    private final Supplier<State> supplier;

    StateFactory (Supplier<State> supplier) {
        this.supplier = supplier;
    }

    public State build() {
        return supplier.get();
    }
}
