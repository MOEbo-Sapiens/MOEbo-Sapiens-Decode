package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.States.IntakingState;

import java.util.function.Supplier;

public enum StateBuilder {
    INTAKING(IntakingState::new);

    private final Supplier<State> supplier;

    StateBuilder (Supplier<State> supplier) {
        this.supplier = supplier;
    }

    public State build() {
        return supplier.get();
    }
}
