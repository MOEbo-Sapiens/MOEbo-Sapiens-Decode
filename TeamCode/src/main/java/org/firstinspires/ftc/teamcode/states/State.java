package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;

public interface State {

    void initialize(Robot robot, State prevState);
    void execute(Robot robot);
    String name();
}