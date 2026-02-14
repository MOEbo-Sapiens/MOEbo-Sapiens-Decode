package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Proximity Indicator Test", group = "Test")
public class ProximityIndicator extends LinearOpMode {

    private org.firstinspires.ftc.teamcode.robot.ProximityIndicator proximityIndicator;
    private DistanceSensor distanceSensor;

    private static final double DETECTION_DISTANCE_CM = 4.0;

    @Override
    public void runOpMode() {
        proximityIndicator = new org.firstinspires.ftc.teamcode.robot.ProximityIndicator(hardwareMap);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "color");

        telemetry.addLine("Proximity Indicator Test Ready");
        telemetry.addLine("Using goBILDA RGB Indicator Light");
        telemetry.addLine("Pass objects in front of sensor");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            proximityIndicator.update();

            double distanceCm = distanceSensor.getDistance(DistanceUnit.CM);
            boolean detected = distanceCm < DETECTION_DISTANCE_CM;

            telemetry.addData("Distance (cm)", "%.2f", distanceCm);
            telemetry.addData("Object Detected", detected);
            telemetry.addData("Object Count", proximityIndicator.getObjectCount());
            telemetry.addData("Full (3 objects)", proximityIndicator.isFull());
            telemetry.addData("Servo Position", "%.2f", proximityIndicator.getColorPosition());
            telemetry.addLine();
            telemetry.addLine("Color Positions (adjust if needed):");
            telemetry.addData("  RED", "0.00");
            telemetry.addData("  ORANGE", "0.08");
            telemetry.addData("  YELLOW", "0.17");
            telemetry.addData("  GREEN", "0.38");
            telemetry.update();
        }
    }
}
