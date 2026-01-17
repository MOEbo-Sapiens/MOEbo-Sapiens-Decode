package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

public abstract class Auto extends LinearOpMode {
    Robot robot;
    //default for all poses is blue side
    Pose startPose = new Pose(12, 132, Math.toRadians(-45)); //TODO: actually determine
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
