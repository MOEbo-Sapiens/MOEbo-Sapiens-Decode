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
