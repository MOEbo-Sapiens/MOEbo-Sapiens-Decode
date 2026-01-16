package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

public abstract class Tele extends LinearOpMode {
    Robot robot;
    Pose goalPose = Constants.BLUE_GOAL_POSE;
    //default startPose
    Pose startPose = new Pose(0, 0, Math.toRadians(0)); //TODO: actually determine

    public void initialize() {
        setPoses();
        Scheduler.reset();

        if (!Constants.lastOpModeWasAuto) {
            robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry, goalPose);
            robot.setPose(startPose);
        } else {
            robot = Constants.robot;
        }
       Constants.lastOpModeWasAuto = false;

        robot.init();
        Scheduler.schedule(
            infinite(robot::updateDrive)
        );
    }

    abstract void setPoses();

    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        while (opModeIsActive()) {
            Scheduler.execute();
        }
    }
}
