package org.firstinspires.ftc.teamcode.shooter;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import smile.interpolation.LinearInterpolation;

import org.firstinspires.ftc.teamcode.robot.Constants;

public class Turret {
    DcMotorEx turretMotor;
    private double cachedPower = 0;

    public static double kP = 0, kD = 0, kF = 0; //TODO: Tune
    PIDFController turretPIDF = new PIDFController(new PIDFCoefficients(kP, 0, kD, kF));
    private boolean activated = false;
    private double targetTicks = 0;

    public static double MIN_TURRET_ANGLE = Math.toRadians(-180);
    public static double MAX_TURRET_ANGLE = Math.toRadians(180);

    public static double MIN_TURRET_TICKS = -1000; //TODO: TUNE
    public static double MAX_TURRET_TICKS = 1000; //TODO: TUNE

    double[] angleValues = new double[] {MIN_TURRET_ANGLE, MAX_TURRET_ANGLE};
    double[] tickValues = new double[] {MIN_TURRET_TICKS, MAX_TURRET_TICKS};

    LinearInterpolation angleToTicks = new LinearInterpolation(angleValues, tickValues);
    LinearInterpolation ticksToAngle = new LinearInterpolation(tickValues, angleValues);

    public Turret(HardwareMap hardwareMap) {
        this.turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        reset();
    }

    private void reset() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power) {
        if (Math.abs(power - cachedPower) < Constants.MOTOR_POWER_THRESHOLD) {
            return;
        }
        cachedPower = power;
        turretMotor.setPower(power);
    }

    private double getCurrentPositionTicks() {
        return turretMotor.getCurrentPosition();
    }

    /**
     * @return The current angle of the turret in radians. Note that counter clockwise is positive with 0 being straight forward.
     */
    public double getCurrentAngle() {
        double currentTicks = getCurrentPositionTicks();
        return ticksToAngle.interpolate(currentTicks);
    }

    public double getTargetAngle() {
        return ticksToAngle.interpolate(targetTicks);
    }

    private void setTargetTicks(double ticks) {
        this.targetTicks = ticks;
    }

    public void setTurretAngle(double radians) {
        setTargetTicks(
            angleToTicks.interpolate(radians)
        );
    }

    public void activate()  {
        activated = true;
    }

    public void deactivate() {
        activated = false;
        setPower(0);
    }

    public void toggle() {
        activated = !activated;
        if (!activated) {
            setPower(0);
        }
    }

    public void update() {
        if (activated) {
            double currentTicks = getCurrentPositionTicks();
            turretPIDF.updatePosition(currentTicks);
            turretPIDF.setTargetPosition(targetTicks);
            double power = turretPIDF.run();
            setPower(power);
        }
    }
}
