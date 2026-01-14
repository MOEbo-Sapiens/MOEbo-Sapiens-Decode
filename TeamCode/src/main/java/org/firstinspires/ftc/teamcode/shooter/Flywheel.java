package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.Constants;

public class Flywheel {
    private DcMotorEx shooterMotorL;
    private DcMotorEx shooterMotorR;
    private double cachedPower = 0;

    public static final double TICKS_PER_REV = 28;

    public static double kS = 0.08, kV = 0.00039, kP = 0.01; //TODO: TUNE VALUES, STOLEN FROM BARON

    private double target = 0;
    private boolean activated = false;

    public Flywheel(HardwareMap hardwareMap) {
        shooterMotorL = hardwareMap.get(DcMotorEx.class, "Shooter Motor L");
        shooterMotorR = hardwareMap.get(DcMotorEx.class, "Shooter Motor R");

        shooterMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotorR.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    public double getCurrentVelocityInTicks() {
        return shooterMotorR.getVelocity();
    }

    public double getCurrentVelocityInRPS() {
        return getCurrentVelocityInTicks() / TICKS_PER_REV;
    }

    public double getCurrentAngularVelocity() {
        return getCurrentVelocityInRPS();
    }

    public void setTargetVelInTicks(double target) {
        this.target = target;
    }

    public void setTargetAngularVelocity(double targetRPS) {
        setTargetVelInTicks(targetRPS * TICKS_PER_REV);
    }

    public void setTargetRPS(double targetRPS) {
        setTargetVelInTicks(targetRPS * TICKS_PER_REV);
    }

    public double getTargetInTicks() {
        return target;
    }

    public double getTargetRPS() {
        return getTargetInTicks() / TICKS_PER_REV;
    }

    public double getTargetAngularVelocity() {
        return getTargetRPS();
    }

    public void setPower(double power) {
        if (Math.abs(power - cachedPower) < Constants.MOTOR_POWER_THRESHOLD) {
            return;
        }
        cachedPower = power;
        shooterMotorL.setPower(power);
        shooterMotorR.setPower(power);
    }

    public void deactivate() {
       activated = false;
       setPower(0);
    }

    public void activate() {
        activated = true;
    }

    public void toggle() {
        activated = !activated;
        if (!activated) {
            setPower(0);
        }
    }

    public void update() {
        if (activated) {
            setPower((kV * getTargetInTicks()) + (kP * (getTargetInTicks() - getCurrentVelocityInTicks())) + kS);
        }
    }







}
