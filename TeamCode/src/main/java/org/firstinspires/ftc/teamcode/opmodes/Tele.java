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
    Pose startPose = new Pose(48, (12.0459149606/2), Math.toRadians(90)); //TODO: actually determine
    //length of dt: 305.86624, width: 311.4


    public void initialize() {
        setPoses();
        Scheduler.reset();

        if (!Constants.lastOpModeWasAuto) {
            Constants.reset();
            robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry, goalPose);
            robot.setPose(startPose);
        } else {
            robot = Constants.robot;
        }

        setColor();

       Constants.lastOpModeWasAuto = false;

        robot.init();
        Scheduler.schedule(
            infinite(robot::updateDrive)
        );
    }

    abstract void setPoses();

    abstract void setColor();

    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        while (opModeIsActive()) {
            Scheduler.execute();
        }
    }
}
