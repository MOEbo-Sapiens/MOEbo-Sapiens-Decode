package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Proximity Indicator", group = "Test")
public class ProximityIndicator extends LinearOpMode {

    private RevBlinkinLedDriver blinkin;
    private DistanceSensor distanceSensor;

    private static final double DETECTION_DISTANCE_CM = 4.0;

    @Override
    public void runOpMode() {

        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "color");

        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

        waitForStart();

        while (opModeIsActive()) {

            double distanceCm = distanceSensor.getDistance(DistanceUnit.CM);

            if (distanceCm < DETECTION_DISTANCE_CM) {
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } else {
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }

            telemetry.addData("Distance (cm)", "%.2f", distanceCm);
            telemetry.update();
        }
    }
}
