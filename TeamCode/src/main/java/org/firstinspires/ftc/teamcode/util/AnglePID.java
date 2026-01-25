package org.firstinspires.ftc.teamcode.util;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.pedropathing.util.Timer;


public class AnglePID {
    PIDCoefficients coefficients;
    protected boolean hasRun = false;
    protected Timer timer = new Timer();
    protected double previousError = (double)0.0F;
    protected double integralSum = (double)0.0F;
    protected double derivative = (double)0.0F;

    public AnglePID(PIDCoefficients coefficients) {
        this.coefficients = coefficients;
    }

    public double calculate(double reference, double state) {
        double dt = this.getDT();
        double error = MathHelpers.wrapAngleRadians(reference - state);
        double derivative = this.calculateDerivative(error, dt);
        this.integrate(error, dt);
        this.previousError = error;
        return error * this.coefficients.Kp + this.integralSum * this.coefficients.Ki + derivative * this.coefficients.Kd;
    }

    public double getDT() {
        if (!this.hasRun) {
            this.hasRun = true;
            this.timer.resetTimer();
        }

        double dt = this.timer.getElapsedTime();
        this.timer.resetTimer();
        return dt;
    }

    protected void integrate(double error, double dt) {
        this.integralSum += (error + this.previousError) / (double)2.0F * dt;
    }

    protected double calculateDerivative(double error, double dt) {
        this.derivative = (error - this.previousError) / dt;
        return this.derivative;
    }

}