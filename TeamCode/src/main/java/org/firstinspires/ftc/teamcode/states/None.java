package org.firstinspires.ftc.teamcode.states;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.States;

public class None implements State {

    public None(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
       //do nothing
    }
    @Override
    public void initialize(Robot robot, State prevState) {
       //do nothing
    }

    @Override
    public void execute(Robot robot) {
        //do nothing
    }

    @Override
    public String name() {
        return "None";
    }
}
