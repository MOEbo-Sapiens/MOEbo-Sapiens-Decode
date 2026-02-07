package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Constants;


@TeleOp
public class SetLastTurretTicks extends LinearOpMode {

    public void runOpMode() {
        waitForStart();
        Constants.setLastTurretTicks(100);

        while (opModeIsActive()) {
            telemetry.addData("lastTurretTicks", Constants.getLastTurretTicks());
            telemetry.update();
        }
    }
}
