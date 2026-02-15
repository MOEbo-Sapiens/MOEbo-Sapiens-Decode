package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestTurret {
    private DcMotor turretMotor;
    private double cachedPower = 0;
    private final double turretMotorPower = 0.9;
    private final double kP = 0.1;

//    private BasicPID controller;

    private int target_tick = 0;
    private final int MAX_TICKS = 2151;
    Telemetry telemetry;

    public TestTurret(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

//        controller = new BasicPID(new PIDCoefficients(kP, 0, 0));

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);

        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void turnToAngle(double angle) {
        int angle_ticks = (int)((angle/360.0) * MAX_TICKS);
        double headingError = angle_ticks - turretMotor.getCurrentPosition();

        telemetry.addData("Target Ticks:", angle_ticks);
        telemetry.addData("Error Ticks:", headingError);
        telemetry.addData("Current Ticks:", turretMotor.getCurrentPosition());

        turretMotor.setTargetPosition(angle_ticks);
        setPower(turretMotorPower);
    }

    private void setPower(double power) {
        if (Math.abs(power - cachedPower) < Constants.MOTOR_CACHING_TOLERANCE) {
            return;
        }
        cachedPower = power;
        turretMotor.setPower(power);
    }
}
