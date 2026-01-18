package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.Scheduler.cancel;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.States;

public abstract class Auto extends LinearOpMode {
    Robot robot;
    //default for all poses is blue side
    Pose startPose = new Pose(48, (12.0459149606/2), Math.toRadians(90)); //TODO: actually determine
    Pose goalPose = Constants.BLUE_GOAL_POSE;
    //Pose pose1...

    abstract void setPoses();
    abstract void setColor();

    private void createAutoCommands() {
        //TODO: schedule all desired auto commands here
    }

    public void initialize() {
        Constants.reset();
        setColor();
        setPoses();
        Constants.lastOpModeWasAuto = true;
        Scheduler.reset();

        robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry, goalPose);
        Constants.robot = robot;
        robot.setPose(startPose);
        robot.init();
        createAutoCommands();
        waitForStart();
    }

    public void runOpMode() throws InterruptedException {
        initialize();


        waitForStart();
        while (opModeIsActive()) {
            Scheduler.execute();
        }
    }
}
