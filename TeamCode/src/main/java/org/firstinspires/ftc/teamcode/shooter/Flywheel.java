package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Constants;

import java.awt.font.NumericShaper;

@Config
public class Flywheel {
    private DcMotorEx shooterMotorL;
    private DcMotorEx shooterMotorR;
    private double cachedPower = 0;

    public static double kS = 0.08, kV = 0.00039, kP = 0.02; //TODO: TUNE VALUES, STOLEN FROM BARON

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

    /**
     *
     * @param ticksPerSecond motor speed in ticks per second
     * @return flywheel speed in radians per second
     */
    public static double motorTicksToFlywheelRadians(double ticksPerSecond) {
        //28 -- ticks per rotation
        // 1.4 -- ratio of flywheel velocity to motor velocity
        return (ticksPerSecond / 28) * 2 * Math.PI * 1.4;
    }

    /**
     *
     * @param radiansPerSecond flywheel speed in radians per second
     * @return motor speed in ticks per second
     */
    public static double flywheelRadiansToMotorTicks(double radiansPerSecond) {
        //28 -- ticks per rotation
        // 1.4 -- ratio of flywheel velocity to motor velocity
        return (radiansPerSecond / (2 * Math.PI * 1.4)) * 28;
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
            double power =  (kV * getTargetAngularVelocity()) + (kP * (getTargetAngularVelocity() - getCurrentAngularVel())) + kS;
            power = Range.clip(power, -1, 1);
            setPower(power);
        }
    }







}
