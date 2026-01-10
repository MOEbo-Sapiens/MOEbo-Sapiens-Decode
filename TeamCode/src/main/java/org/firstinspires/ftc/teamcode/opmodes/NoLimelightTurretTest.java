package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Turret;

@TeleOp
public class NoLimelightTurretTest extends LinearOpMode {
    private Turret turret;

    @Override
    public void runOpMode() throws InterruptedException {
        turret = new Turret(hardwareMap, telemetry);

        waitForStart();
        while(opModeIsActive()) {
            turret.turnToAngle(90);
            telemetry.update();
        }
    }
}
