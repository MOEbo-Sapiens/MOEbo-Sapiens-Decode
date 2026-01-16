package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.shooter.Hood;

import java.util.List;

@TeleOp
public class ShootingTest extends LinearOpMode {
    Flywheel flywheel;
    Hood hood;
    Intake intake;

    double flywheelTarget = 0;
    double hoodTarget = 0;
    //max speed is 2180 ticks / second -> correlates with roughly 5 radians per second


    //all to red score
    //FAR ZONE:
    //  Speed: 1760, Servo Pos: 0.64, launch angle: 23.7834780759°, hood angle: 66.2167323833°
    //AT THE TOP OF FAR ZONE: 72, 24


    //CLOSE LERP 1:


    //Speed: 1315, servo pos 0.36, launch angle: 35.2598227124°, hood angle: 54.7401772876°

    // Speed 1260, Servo pos 0.35, launch angle 35.6608931689°, hood angle: 54.3393172902°
    // Pose: 72, 72

    //Speed: 1040, Servo pos: 0.1, launch angle 45.9053785459°, hood angle: 44.0946214541°
    //Pose: 96, 96

    //Speed: 1030, Servo Pos: 0.08, launch angle: 46.7247081929°, hood angle: 43.2752918071°
    //Pose: 102, 102


    @Override
    public void runOpMode() throws InterruptedException {
        flywheel = new Flywheel(hardwareMap);
        hood = new Hood(hardwareMap);
        intake = new Intake(hardwareMap);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        flywheel.deactivate();

        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            flywheelTarget += gamepad1.right_trigger;
            flywheelTarget -= gamepad1.left_trigger;

            if (gamepad1.left_bumper) {
                intake.setPower(-1);
            } else if (gamepad1.right_bumper) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            if (gamepad1.aWasPressed()) {
                flywheel.activate();
            }


            if (gamepad1.bWasPressed()) {
                flywheel.deactivate();
            }

            if (gamepad1.dpadUpWasPressed()) {
                hoodTarget += 0.05;
                hoodTarget = Math.min(1, hoodTarget);
            }

            if (gamepad1.dpadRightWasPressed()) {
                hoodTarget += 0.01;
                hoodTarget = Math.min(1, hoodTarget);
            }


            if (gamepad1.dpadDownWasPressed()) {
                hoodTarget -= 0.05;
                hoodTarget = Math.max(0, hoodTarget);
            }

            if (gamepad1.dpadLeftWasPressed()) {
                hoodTarget -= 0.01;
                hoodTarget = Math.max(0, hoodTarget);
            }

            flywheel.setTargetAngularVelocity(flywheelTarget);
            flywheel.update();
            hood.setTargetPosition(hoodTarget);

            telemetry.addData("Current Angular Vel", flywheel.getCurrentAngularVel());
            telemetry.addData("Target Angular Vel", flywheel.getTargetAngularVelocity());
            telemetry.addData("\n\nCurrent Hood angle", hood.getCurrentHoodAngle());
            telemetry.addData("Target Hood angle", hood.getTargetHoodAngle());
            telemetry.addData("\nCurrent Launch angle", hood.getCurrentLaunchAngle());
            telemetry.addData("Target Launch angle", hood.getTargetLaunchAngle());
            telemetry.addData("\nCurrent Hood Servo Position", hoodTarget);
            telemetry.update();
        }
    }
}
