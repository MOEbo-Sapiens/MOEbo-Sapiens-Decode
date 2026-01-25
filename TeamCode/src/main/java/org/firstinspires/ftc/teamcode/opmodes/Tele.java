package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.commands.Commands.infinite;

import android.net.sip.SipSession;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.States;
import org.firstinspires.ftc.teamcode.shooter.Turret;

public abstract class Tele extends LinearOpMode {
    Robot robot;
    Pose goalPose = Constants.BLUE_GOAL_POSE;
    //default startPose
    //length: 15.28
    //width: 15.12
    Pose startPose = new Pose(48, (15.28/2), Math.toRadians(90)); //TODO: actually determine


    public void initialize() {
        setPoses();
        Scheduler.reset();

        if (!Constants.lastOpModeWasAuto) {
            Constants.reset();
            //robot needs to be created after Constants.reset() probably
            robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry, goalPose);
            robot.setPose(startPose);
        } else {
            robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry, goalPose);
            Turret.turretOffset = Constants.lastTurretTicks;
            robot.setPose(Constants.lastPose);
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
        robot.setState(States.INTAKING);
        while (opModeIsActive()) {
            Scheduler.execute();
        }
    }
}
