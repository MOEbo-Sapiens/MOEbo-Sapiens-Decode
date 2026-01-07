package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface State {

    void initialize(Robot robot, IRobot prevState);
    void execute(Robot robot, Telemetry telemetry);
}