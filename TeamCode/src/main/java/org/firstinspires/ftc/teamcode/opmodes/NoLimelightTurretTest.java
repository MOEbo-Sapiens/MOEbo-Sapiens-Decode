package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.TestTurret;

@TeleOp
public class NoLimelightTurretTest extends LinearOpMode {
    private TestTurret testTurret;

    @Override
    public void runOpMode() throws InterruptedException {
        testTurret = new TestTurret(hardwareMap, telemetry);

        waitForStart();
        while(opModeIsActive()) {
            testTurret.turnToAngle(90);
            telemetry.update();
        }
    }
}
