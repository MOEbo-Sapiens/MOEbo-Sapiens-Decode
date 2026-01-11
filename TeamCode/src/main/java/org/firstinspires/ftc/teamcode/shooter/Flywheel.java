package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import smile.interpolation.BilinearInterpolation;
import smile.interpolation.Interpolation2D;

public class Flywheel {
    private DcMotorEx shooterMotorL;
    private DcMotorEx shooterMotorR;

    public static double kS = 0.08, kV = 0.00039, kP = 0.01; //TODO: TUNE VALUES, STOLEN FROM BARON

    private double target = 0;
    boolean activated = false;

    private double[] xValues = new double[3];
    private double[] yValues = new double[3];
    private double[][] closeVelocities = new double[][]{
            new double[3],
            new double[3],
            new double[3]
    };


    Interpolation2D closeVelocitiesInterpolation = new BilinearInterpolation(xValues, yValues, closeVelocities);

    public Flywheel(HardwareMap hardwareMap) {
        shooterMotorL = hardwareMap.get(DcMotorEx.class, "Shooter Motor L");
        shooterMotorR = hardwareMap.get(DcMotorEx.class, "Shooter Motor R");
    }


    public double getCurrentVelocity() {
        return shooterMotorR.getVelocity();
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public double getTarget() {
        return target;
    }

    public void setPower(double power) {
        shooterMotorL.setPower(power);
        shooterMotorR.setPower(power);
    }

    public void off() {
       activated = false;
       setPower(0);
    }

    public void on() {
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
            setPower((kV * getTarget()) + (kP * (getTarget() - getCurrentVelocity())) + kS);
        }
    }







}
