package org.firstinspires.ftc.teamcode.states;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.States;

public class ShootingState implements State {
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;
    public ShootingState(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void initialize(Robot robot, State prevState) {
        schedule(robot.activateShooter());
        schedule(robot.openGate());
    }

    public void execute(Robot robot) {
        schedule(robot.updateShooter());

        if (gamepad1.aWasPressed()) {
            schedule(
                    sequential(
                            waitUntil(robot::readyToShoot).raceWith(infinite(() -> {
                                telemetry.addData("Waiting to shoot...", "");
                                telemetry.addData("Flywheel Velocity", robot.getFlywheelAngularVelocity());
                                telemetry.addData("Hood Angle", robot.getHoodAngleDegrees());
                                telemetry.addData("Turret Angle", robot.getTurretAngleDegrees());
                            })),
                            robot.setIntakePower(1), //TODO: make this adjust for shooting three times (eg only updating turret while shooting but also 3 shots)
                            waitMs(500),
                            robot.setIntakePower(0),
                            instant(() -> robot.setState(States.INTAKING))
                    )
            );
        }
    }

    public String name(){
        return "Shooting";
    }
}
