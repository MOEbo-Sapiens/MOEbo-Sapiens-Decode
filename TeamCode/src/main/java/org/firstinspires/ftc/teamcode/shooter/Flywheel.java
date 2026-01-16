package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Constants;

@Config
public class Flywheel {
    private DcMotorEx shooterMotorL;
    private DcMotorEx shooterMotorR;
    private double cachedPower = 0;

    public static double kS = 0.08, kV = 0.00039, kP = 0.01; //TODO: TUNE VALUES, STOLEN FROM BARON

    private double target = 0;
    private boolean activated = false;

    public Flywheel(HardwareMap hardwareMap) {
        shooterMotorL = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        shooterMotorR = hardwareMap.get(DcMotorEx.class, "rightFlywheel");

        shooterMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotorR.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }


    public double getCurrentAngularVel() {
        return shooterMotorR.getVelocity();
    }

    public void setTargetAngularVelocity(double target) {
       this.target = target;
    }


    public double getTargetAngularVelocity() {
        return target;
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
            setPower((kV * getTargetAngularVelocity()) + (kP * (getTargetAngularVelocity() - getCurrentAngularVel())) + kS);
        }
    }







}
