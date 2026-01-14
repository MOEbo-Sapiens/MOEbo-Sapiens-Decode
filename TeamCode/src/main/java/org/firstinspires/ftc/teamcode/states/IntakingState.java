package org.firstinspires.ftc.teamcode.states;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.bindings.Binding;
import com.pedropathing.ivy.bindings.Bindings;
import com.qualcomm.robotcore.hardware.Gamepad;

import static com.pedropathing.ivy.bindings.Bindings.bind;
import static com.pedropathing.ivy.commands.Commands.instant;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.States;

public class IntakingState implements State {

    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;
    public IntakingState(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void initialize(Robot robot, State prevState) {
        schedule(robot.closeGate());
        schedule(robot.deactivateShooter());
    }

    public void execute(Robot robot) {
        if (gamepad1.aWasPressed() && !Constants.lastOpModeWasAuto) {
            schedule(
                    robot.setIntakePower(0),
                    instant(() -> robot.setState(States.SHOOTING))
            );
        }

        if (gamepad1.right_trigger > 0) {
            schedule(robot.setIntakePower(gamepad1.right_trigger));
        }

        if (gamepad1.left_trigger > 0) {
            schedule(robot.setIntakePower(-gamepad1.left_trigger));
        }
    }

    public String name(){
        return "Intaking";
    }
}
