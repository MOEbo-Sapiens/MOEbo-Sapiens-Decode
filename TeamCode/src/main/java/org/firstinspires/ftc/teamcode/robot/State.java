package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface State {

    void initialize(Robot robot, State prevState, Telemetry telemetry);
    void execute(Robot robot);
}