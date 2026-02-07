package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Constants;

@TeleOp
public class GetLastTurretTicks extends LinearOpMode {

    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("lastTurretTicks", Constants.getLastTurretTicks());
            telemetry.update();
        }
    }
}
