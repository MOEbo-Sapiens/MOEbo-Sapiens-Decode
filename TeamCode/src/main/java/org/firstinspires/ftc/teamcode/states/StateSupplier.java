package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@FunctionalInterface
public interface StateSupplier {
    State get(Telemetry telemetry);
}
