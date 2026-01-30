package org.firstinspires.ftc.teamcode.shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.Tele;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.shooter.math.VelocityCompensationCalculator;
import org.firstinspires.ftc.teamcode.util.MathHelpers;

import smile.interpolation.Interpolation;
import smile.interpolation.LinearInterpolation;

/**
 * Shooter class using simplified VelocityCompensationCalculator
 */
public class Shooter {

    public static double transitionYValue = 40;

    public static double distance(Pose launchPose, Pose goalPose) {
        return Math.hypot(goalPose.getX() - launchPose.getX(), goalPose.getY() - launchPose.getY());
    }

    // Fallback interpolation tables (used when velocity comp is off)
    private double[] closeDistances = new double[] {
            distance(new Pose(48, 96), Constants.BLUE_GOAL_POSE.mirror()),
            distance(new Pose(72, 72), Constants.BLUE_GOAL_POSE.mirror()),
            distance(new Pose(88, 85), Constants.BLUE_GOAL_POSE.mirror()),
            distance(new Pose(90, 90), Constants.BLUE_GOAL_POSE.mirror()),
            distance(new Pose(96, 96), Constants.BLUE_GOAL_POSE.mirror()),
            distance(new Pose(102, 102), Constants.BLUE_GOAL_POSE.mirror()),
    };

    private double[] closeSpeeds = new double[] { 1315, 1220, 1120, 1078, 1034, 1006 };
    private double[] closeAngles = new double[] {
            Math.toRadians(54.74), Math.toRadians(42.05), Math.toRadians(40.0),
            Math.toRadians(40.0), Math.toRadians(40.0), Math.toRadians(40.0)
    };

    private double[] farDistances = new double[] {
            distance(new Pose(72, 24), Constants.BLUE_GOAL_POSE),
            distance(new Pose(84, 12), Constants.BLUE_GOAL_POSE)
    };
    private double[] farSpeeds = new double[] { 1712, 1712 };
    private double[] farAngles = new double[] { Math.toRadians(64.17), Math.toRadians(61.71) };

    Interpolation closeFlywheelSpeeds = new LinearInterpolation(closeDistances, closeSpeeds);
    Interpolation closeHoodAngles = new LinearInterpolation(closeDistances, closeAngles);
    Interpolation farFlywheelSpeeds = new LinearInterpolation(farDistances, farSpeeds);
    Interpolation farHoodAngles = new LinearInterpolation(farDistances, farAngles);

    // Tolerances
    public static int flywheelToleranceTicks = 60;
    public static double turretToleranceDegrees = 6.7;
    public static double hoodToleranceDegrees = 2;

    // Gate positions
    public static double openGatePosition = 0.3;
    public static double closedGatePosition = 0.11;

    // Hardware
    Hood hood;
    Flywheel flywheel;
    Turret turret;
    Servo gateServo;
    Follower follower;
    Pose goalPose;

    public double lastTurretAngle;

    public Shooter(HardwareMap hardwareMap, Follower follower, Pose goalPose) {
        hood = new Hood(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        turret = new Turret(hardwareMap);
        gateServo = hardwareMap.get(Servo.class, "gate");
        this.goalPose = goalPose;
        this.follower = follower;
    }

    /**
     * Update shooting subsystems WITH velocity compensation
     */
    public void updateShootingSubsystems(Pose pose, Telemetry telemetry, boolean useVelocityComp) {
        if (!useVelocityComp) {
            updateShootingSubsystems(pose, telemetry);
            return;
        }

        boolean close = pose.getY() > transitionYValue;

        // Get robot velocity from follower
        Vector translationalVel = follower.getVelocity();
        Pose velocity = new Pose(
                translationalVel.getXComponent(),
                translationalVel.getYComponent(),
                follower.getAngularVelocity()
        );

        // Calculate shot parameters using the simplified approach
        VelocityCompensationCalculator.ShotParameters params =
                VelocityCompensationCalculator.calculate(
                        pose, velocity,
                        goalPose,
                        close
                );

        // Check validity and fall back if needed
        if (!params.isValid) {
            telemetry.addData("VelComp FAILED", params.errorMessage);
            telemetry.addData("Falling back to", "no velocity comp");
            updateShootingSubsystems(pose, telemetry);
            return;
        }

        lastTurretAngle = params.turretAngle;

        // Debug telemetry
        telemetry.addData("Zone", close ? "CLOSE" : "FAR");
        telemetry.addData("flywheelSpeed", params.flywheelTicks);
        telemetry.addData("hoodAngle", Math.toDegrees(params.hoodAngle));
        telemetry.addData("Turret Angle", Math.toDegrees(params.turretAngle));
        telemetry.addData("Vrr", String.format("%.1f in/s", params.vrr));
        telemetry.addData("Vrt", String.format("%.1f in/s", params.vrt));
        telemetry.addData("Vx_new", String.format("%.1f in/s", params.vxNew));
        telemetry.addData("Launch V", String.format("%.1f in/s", params.launchVelocity));
        telemetry.addData("Launch α", String.format("%.1f°", Math.toDegrees(params.launchAngle)));

        // Command hardware
        flywheel.setTargetAngularVelocity(params.flywheelTicks);
        hood.setHoodAngle(params.hoodAngle);
        turret.setTurretAngle(params.turretAngle);
    }

    /**
     * Update turret only during intaking state
     */
    public void updateTurretOnly(Pose pose, Telemetry telemetry, boolean useVelocityComp) {
        if (!useVelocityComp) {
            updateTurretOnly(pose, telemetry);
            return;
        }

        boolean close = pose.getY() > transitionYValue;

        // Get robot velocity from follower
        Vector translationalVel = follower.getVelocity();
        Pose velocity = new Pose(
                translationalVel.getXComponent(),
                translationalVel.getYComponent(),
                follower.getAngularVelocity()
        );

        // Calculate shot parameters using the simplified approach
        VelocityCompensationCalculator.ShotParameters params =
                VelocityCompensationCalculator.calculate(
                        pose, velocity,
                        goalPose,
                        close
                );

        // Check validity and fall back if needed
        if (!params.isValid) {
            telemetry.addData("VelComp FAILED", params.errorMessage);
            telemetry.addData("Falling back to", "no velocity comp");
            updateShootingSubsystems(pose, telemetry);
            return;
        }

        lastTurretAngle = params.turretAngle;

        // Debug telemetry
        telemetry.addData("Turret Angle", Math.toDegrees(params.turretAngle));
        telemetry.addData("Goal Pose", goalPose);

        turret.setTurretAngle(params.turretAngle);
        flywheel.setPower(0);
    }

    public void updateTurretOnly(Pose pose, Telemetry telemetry) {
        boolean close = pose.getY() > transitionYValue;

        Interpolation flywheelInterpolation = close ? closeFlywheelSpeeds : farFlywheelSpeeds;
        Interpolation hoodAngleInterpolation = close ? closeHoodAngles : farHoodAngles;

        double dist = distance(pose, goalPose);
        double flywheelSpeed = flywheelInterpolation.interpolate(dist);
        double hoodAngle = hoodAngleInterpolation.interpolate(dist);
        double turretAngle = getTargetTurretAngle(pose);

        telemetry.addData("Turret Angle", Math.toDegrees(turretAngle));
        telemetry.addData("Goal Pose", goalPose);

        lastTurretAngle = turretAngle;

        turret.setTurretAngle(turretAngle);
    }

    /**
     * Update shooting subsystems WITHOUT velocity compensation (fallback/simple mode)
     */
    public void updateShootingSubsystems(Pose pose, Telemetry telemetry) {
        boolean close = pose.getY() > transitionYValue;

        Interpolation flywheelInterpolation = close ? closeFlywheelSpeeds : farFlywheelSpeeds;
        Interpolation hoodAngleInterpolation = close ? closeHoodAngles : farHoodAngles;

        double dist = distance(pose, goalPose);
        double flywheelSpeed = flywheelInterpolation.interpolate(dist);
        double hoodAngle = hoodAngleInterpolation.interpolate(dist);
        double turretAngle = getTargetTurretAngle(pose);

        telemetry.addData("flywheelSpeed", flywheelSpeed);
        telemetry.addData("hoodAngle", Math.toDegrees(hoodAngle));
        telemetry.addData("Turret Angle", Math.toDegrees(turretAngle));
        telemetry.addData("Goal Pose", goalPose);

        lastTurretAngle = turretAngle;

        flywheel.setTargetAngularVelocity(flywheelSpeed);
        hood.setHoodAngle(hoodAngle);
        turret.setTurretAngle(turretAngle);
    }

    public double getTargetTurretAngle(Pose pose) {
        double targetAngle = Math.atan2(
                goalPose.getY() - pose.getY(),
                goalPose.getX() - pose.getX()
        );
        targetAngle -= pose.getHeading();
        targetAngle = MathHelpers.wrapAngleRadians(targetAngle);
        return targetAngle;
    }

    public boolean readyToShoot() {
        return Math.abs(flywheel.getTargetAngularVelocity() - flywheel.getCurrentAngularVel()) < flywheelToleranceTicks &&
                Math.abs(turret.getTargetAngle() - turret.getCurrentAngle()) < Math.toRadians(turretToleranceDegrees) &&
                Math.abs(hood.getTargetHoodAngle() - hood.getCurrentHoodAngle()) < Math.toRadians(hoodToleranceDegrees);
    }

    public void activate() {
        flywheel.activate();
        turret.activate();
    }

    public void intakingPos() {
        flywheel.deactivate();
        turret.setTurretAngle(0);
        hood.setHoodAngle(VelocityCompensationCalculator.getMinHoodAngle());
    }

    public void deactivate() {
        flywheel.deactivate();
        turret.deactivate();
    }

    public void deactivateFlywheel() {
        flywheel.deactivate();
    }

    public void toggle() {
        flywheel.toggle();
        turret.toggle();
    }

    public void update(Telemetry telemetry) {
        flywheel.update();
        turret.update(telemetry);
    }

    public void setOpenGatePosition() { gateServo.setPosition(openGatePosition); }
    public void setCloseGatePosition() { gateServo.setPosition(closedGatePosition); }

    public int getTurretTicks() { return turret.getCurrentPositionTicks(); }
    public double getTurretAngle() { return turret.getCurrentAngle(); }
    public double getFlywheelAngularVelocity() { return flywheel.getCurrentAngularVel(); }
    public double getFlywheeelTargetAngularVelocity() { return flywheel.getTargetAngularVelocity(); }
    public double getHoodAngle() { return hood.getCurrentHoodAngle(); }
}
