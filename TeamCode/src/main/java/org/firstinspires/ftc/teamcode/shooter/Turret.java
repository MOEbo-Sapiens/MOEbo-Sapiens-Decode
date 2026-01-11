package org.firstinspires.ftc.teamcode.shooter;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.Constants;

import smile.interpolation.LinearInterpolation;

public class Turret {
    DcMotorEx turretMotor;

    public static double kP = 0, kD = 0, kF = 0; //TODO: Tune
    PIDFController turretPIDF = new PIDFController(new PIDFCoefficients(kP, 0, kD, kF));
    boolean activated = false;
    double targetTicks = 0;

    double[] angleValues = new double[] {Constants.MIN_TURRET_ANGLE, Constants.MAX_TURRET_ANGLE};
    double[] tickValues = new double[] {Constants.MIN_TURRET_TICKS, Constants.MAX_TURRET_TICKS};

    LinearInterpolation angleToTicks = new LinearInterpolation(angleValues, tickValues);
    LinearInterpolation ticksToAngle = new LinearInterpolation(tickValues, angleValues);

    public Turret(HardwareMap hardwareMap) {
        this.turretMotor = hardwareMap.get(DcMotorEx.class, "Turret Motor");
        reset();
    }

    private void reset() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setPower(double power) {
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

    private void setTargetTicks(double ticks) {
        this.targetTicks = ticks;
    }

    public void setAngle(double radians) {
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
            turretPIDF.updatePosition(targetTicks);
            turretPIDF.setTargetPosition(targetTicks);
            double power = turretPIDF.run();
            setPower(power);
        }
    }
}
