package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class IntakingState implements State {

    Telemetry telemetry;
    public IntakingState(Telemetry telemetry) {
       this.telemetry = telemetry;
    }

    public void initialize(Robot robot, State prevState) {
        //TODO: implement
    }

    public void execute(Robot robot) {
        //TODO: implement
    }

    public String name(){
        return "Intaking";
    }
}
