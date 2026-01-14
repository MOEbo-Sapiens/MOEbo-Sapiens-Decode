package org.firstinspires.ftc.teamcode.states;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;

@FunctionalInterface
public interface StateSupplier {
    State get(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2);
}
