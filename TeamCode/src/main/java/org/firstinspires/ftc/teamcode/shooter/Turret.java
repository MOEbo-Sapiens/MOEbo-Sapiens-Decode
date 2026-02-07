package org.firstinspires.ftc.teamcode.shooter;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import smile.interpolation.LinearInterpolation;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Constants;

@Config
public class Turret {

    public static int turretOffset = 0;

    DcMotorEx turretMotor;
    private double cachedPower = 0;

    public static double kP = 0.01, kD = 0.01;
    BasicPID turretPIDF = new BasicPID(new PIDCoefficients(kP, 0, kD));
    private boolean activated = false;
    private double targetTicks = 0;

    public static double MIN_TURRET_ANGLE = Math.toRadians(-90);
    public static double MAX_TURRET_ANGLE = Math.toRadians(90);

    double[] angleValues = new double[] {Math.toRadians(-90), Math.toRadians(90)};
    double[] tickValues = new double[] {-453, 453};

    LinearInterpolation angleToTicks = new LinearInterpolation(angleValues, tickValues);
    LinearInterpolation ticksToAngle = new LinearInterpolation(tickValues, angleValues);

    public double MIN_TURRET_TICKS = angleToTicks.interpolate(MIN_TURRET_ANGLE);
    public double MAX_TURRET_TICKS = angleToTicks.interpolate(MAX_TURRET_ANGLE);

    public Turret(HardwareMap hardwareMap) {
        this.turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        //make sure brake
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public int getCurrentPositionTicks() {
        return turretMotor.getCurrentPosition() + turretOffset;
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

    public double getTargetTicks() {return targetTicks;}

    private void setTargetTicks(double ticks) {
        this.targetTicks = ticks;
    }

    public void setTurretAngle(double radians) {
        setTargetTicks(
                Range.clip(angleToTicks.interpolate(radians),
                        MIN_TURRET_TICKS,
                        MAX_TURRET_TICKS
                )
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


    public void update(Telemetry telemetry) {
        //TODO: Comment below out after tuning
//        BasicPID turretPIDF = new BasicPID(new PIDCoefficients(kP, 0, kD));
        if (activated) {
            double currentTicks = getCurrentPositionTicks();
            double power = turretPIDF.calculate(targetTicks, currentTicks);
            telemetry.addData("error", targetTicks - currentTicks);
            telemetry.addData("power", power);
            setPower(power);
        }
    }
}
