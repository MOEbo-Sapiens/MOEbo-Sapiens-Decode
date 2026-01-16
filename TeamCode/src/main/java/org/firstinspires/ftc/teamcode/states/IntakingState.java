package org.firstinspires.ftc.teamcode.states;

import static com.pedropathing.ivy.Scheduler.cancel;
import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.ivy.Command;
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
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    private Command joystickToIntake;

    private boolean transitioning = false;

    public IntakingState(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void initialize(Robot robot, State prevState) {
        schedule(robot.closeGate());
        schedule(robot.deactivateShooter());

        joystickToIntake = robot.joysticksToIntakePower(
                () -> gamepad1.left_trigger,
                () -> gamepad1.right_trigger
        );

        schedule(joystickToIntake);
        transitioning = false;
    }

    public void execute(Robot robot) {
        if (gamepad1.aWasPressed() && !Constants.lastOpModeWasAuto && !transitioning) {
            transitioning = true;
            cancel(joystickToIntake);
            schedule(
                    robot.setIntakePower(0),
                    instant(() -> robot.setState(States.SHOOTING))
            );
        }
    }

    public String name(){
        return "Intaking";
    }
}
