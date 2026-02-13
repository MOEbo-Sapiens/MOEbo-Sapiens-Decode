package org.firstinspires.ftc.teamcode.shooter.math;

import static org.firstinspires.ftc.teamcode.shooter.Shooter.distance;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.util.MathHelpers;

import smile.interpolation.LinearInterpolation;

import java.util.stream.IntStream;

@Config
public class VelocityCompensationCalculator {

    public static double kRad = 40;
    public static double kTan = 80;

    private static final double g = 386.0885; // in / s^2

    /** Shooter offset from robot center (inches) */
    private static final double SHOOTER_OFFSET_X = -2.42;
    private static final double SHOOTER_OFFSET_Y = 0.0;

    /** HOOD ANGLE LIMITS */
    public static final double MIN_HOOD_ANGLE = Math.toRadians(40);
    public static final double MAX_HOOD_ANGLE = Math.toRadians(74);


    public static final double[] DISTANCES = new double[] {
           distance(new Pose(), Constants.BLUE_GOAL_POSE) - SHOOTER_OFFSET_X,
    };

    public static final double[] SPEEDS = new double[] {

    };

    public static final double[] HOODS = new double[] {

    };

    // Interpolators
    public static LinearInterpolation speedLerp;
    public static LinearInterpolation hoodLerp;
    public static LinearInterpolation vxToDistanceLerp;

    public static final double[] VXS = IntStream.range(0, SPEEDS.length)
            .mapToDouble(i -> SPEEDS[i] * Math.sin(HOODS[i])).toArray();

    public static final double[] SORTED_VXS;
    public static final double[] SORTED_VX_DISTANCES;

    static {
        // Sort VXS and corresponding DISTANCES by VX for the vxToDistance lerp
        Integer[] vxIndices = IntStream.range(0, VXS.length).boxed().toArray(Integer[]::new);
        java.util.Arrays.sort(vxIndices, java.util.Comparator.comparingDouble(i -> VXS[i]));
        SORTED_VXS = java.util.Arrays.stream(vxIndices).mapToDouble(i -> VXS[i]).toArray();
        SORTED_VX_DISTANCES =
                java.util.Arrays.stream(vxIndices).mapToDouble(i -> DISTANCES[i]).toArray();

        speedLerp = new LinearInterpolation(DISTANCES, SPEEDS);
        hoodLerp = new LinearInterpolation(DISTANCES, HOODS);
        vxToDistanceLerp = new LinearInterpolation(SORTED_VXS, SORTED_VX_DISTANCES);
    }

    public static class ShotParameters {
        public final double hoodAngle; // radians
        public final double turretAngle; // radians (robot frame)
        public final double flywheelTicks; // motor ticks/s

        public ShotParameters(double hoodAngle, double turretAngle, double flywheelTicks) {
            this.hoodAngle = hoodAngle;
            this.turretAngle = turretAngle;
            this.flywheelTicks = flywheelTicks;
        }

        public ShotParameters() {
            this.hoodAngle = 0;
            this.turretAngle = 0;
            this.flywheelTicks = 0;
        }
    }

    /**
     * Calculate shot parameters with velocity compensation.
     * 
     * @param robotPose Robot position (x, y, heading)
     * @param robotVel Robot velocity (vx, vy in in/s, omega in rad/s)
     * @param goalPose Goal Position (x,y)
     */
    public static ShotParameters calculate(Pose robotPose, Vector robotVel, Pose goalPose) {

        // shooter offset
        double cosH = Math.cos(robotPose.getHeading());
        double sinH = Math.sin(robotPose.getHeading());

        double shooterX = robotPose.getX() + SHOOTER_OFFSET_X * cosH - SHOOTER_OFFSET_Y * sinH;
        double shooterY = robotPose.getY() + SHOOTER_OFFSET_X * sinH + SHOOTER_OFFSET_Y * cosH;


        // distance to goal with offset
        double dx = goalPose.getX() - shooterX;
        double dy = goalPose.getY() - shooterY;
        double distance = Math.sqrt(dx * dx + dy * dy);
        double initial = speedLerp.interpolate(distance) * Math.sin(hoodLerp.interpolate(distance));
        double angleToGoal = Math.atan2(dy, dx);

        Vector shootingVector = new Vector(initial, angleToGoal);
        Vector tangentVector = new Vector(1, angleToGoal + Math.PI / 2);

        double tof = distance / initial;

        // pose + tof * (v_rad * k_rad * v_tan * k_tan)
        // it doesn't matter that tof has the wrong units because of hte scaling factors k_tan and
        // k_rad

        // get components relative to goal for scaling
        double vRad = robotVel.dot(new Vector(1, angleToGoal));
        double vTan = robotVel.dot(tangentVector);

        double scaledVRad = vRad * tof * kRad;
        double scaledVTan = vTan * tof * kTan;

        Vector correctionVector = new Vector();
        correctionVector.setOrthogonalComponents(scaledVRad, scaledVTan);

        // convert back to field space
        correctionVector.rotateVector(angleToGoal);

        Vector correctedShootingVector = shootingVector.minus(correctionVector);

        double targetDistanceFromVx = vxToDistanceLerp.interpolate(correctedShootingVector.getMagnitude());
        double targetAngle = correctedShootingVector.getTheta();

        double flywheelSpeed = speedLerp.interpolate(targetDistanceFromVx);
        double hoodAngle = hoodLerp.interpolate(targetDistanceFromVx);
        double turretAngle = MathHelpers.wrapAngleRadians(targetAngle - robotPose.getHeading());

        return new ShotParameters(hoodAngle, turretAngle, flywheelSpeed);
    }


    public static double hoodAngleToLaunchAngle(double hoodAngle) {
        return Math.PI / 2 - hoodAngle;
    }

    public static double getMinHoodAngle() {
        return MIN_HOOD_ANGLE;
    }

    public static double getMaxHoodAngle() {
        return MAX_HOOD_ANGLE;
    }
}
