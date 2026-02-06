package org.firstinspires.ftc.teamcode.shooter.math;

import static org.firstinspires.ftc.teamcode.shooter.Shooter.distance;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.util.MathHelpers;

import smile.interpolation.LinearInterpolation;

/**
 * VelocityCompensationCalculator
 * 
 * Hybrid approach:
 * 1. Use your calibrated interpolation tables for base flywheel speed and hood angle
 * 2. Add velocity compensation for turret offset and velocity adjustment
 * 
 * This matches what Team 23435 did - empirical calibration + velocity compensation.
 */
public class VelocityCompensationCalculator {

    // =========================================================================
    // PHYSICAL CONSTANTS
    // =========================================================================

    /** Gravity in inches/s² */
    private static final double g = 386.0885;

    /** Shooter offset from robot center (inches) */
    private static final double SHOOTER_OFFSET_X = -2.42;
    private static final double SHOOTER_OFFSET_Y = 0.0;

    // =========================================================================
    // HOOD ANGLE LIMITS
    // =========================================================================

    public static final double MIN_HOOD_ANGLE = Math.toRadians(40);
    public static final double MAX_HOOD_ANGLE = Math.toRadians(74);

    // =========================================================================
    // CALIBRATION DATA (from your existing tuning)
    // =========================================================================

    // Close zone calibration
    public static final double[] CLOSE_DISTANCES = new double[] {
            distance(new Pose(31.63, 134.60), Constants.BLUE_GOAL_POSE.mirror())+2.42,
            distance(new Pose(47.9, 102.78), Constants.BLUE_GOAL_POSE.mirror())+2.42,
            distance(new Pose(66.53, 74.4), Constants.BLUE_GOAL_POSE.mirror())+2.42,
            distance(new Pose(79.464, 78.384), Constants.BLUE_GOAL_POSE.mirror())+2.42,
            distance(new Pose(82.666, 82.196), Constants.BLUE_GOAL_POSE.mirror())+2.42,
            distance(new Pose(84.74, 86.23), Constants.BLUE_GOAL_POSE.mirror())+2.42,
            distance(new Pose(90.404, 90.1075), Constants.BLUE_GOAL_POSE.mirror())+2.42,
            distance(new Pose(94.675, 97.423), Constants.BLUE_GOAL_POSE.mirror())+2.42,
            distance(new Pose(104.37, 105.868), Constants.BLUE_GOAL_POSE.mirror())+2.42,
    };
    public static final double[] CLOSE_SPEEDS = {
            1283,
            1238,
            1216,
            1161,
            1118,
            1030,
            1013,
            993,
            980};
    public static final double[] CLOSE_HOODS = {
            Math.toRadians(49.423339408),
            Math.toRadians(48.1914801485),
            Math.toRadians(46.9653504669),
            Math.toRadians(46.1460208198),
            Math.toRadians(44.9141615603),
            Math.toRadians(40.0),
            Math.toRadians(40.0),
            Math.toRadians(40.0),
            Math.toRadians(40.0)
    };

    // Far zone calibration (add your data)
    public static final double[] FAR_DISTANCES = new double[] {
            distance(new Pose(69.247, 30.88), Constants.BLUE_GOAL_POSE.mirror())+2.42,
            distance(new Pose(81.295, 13), Constants.BLUE_GOAL_POSE)+2.42,
            distance(new Pose(44.33, 18.634), Constants.BLUE_GOAL_POSE)+2.42
    };
    public static final double[] FAR_SPEEDS = {
            1433,
            1493,
            1547
    };
    public static final double[] FAR_HOODS = {
            Math.toRadians(53.1074580307),
            Math.toRadians(55.9779765843),
            Math.toRadians(54.7461173248)
    };

    // Interpolators
    public static LinearInterpolation closeSpeedLerp;
    public static LinearInterpolation closeHoodLerp;
    public static LinearInterpolation farSpeedLerp;
    public static LinearInterpolation farHoodLerp;

    static {
        closeSpeedLerp = new LinearInterpolation(CLOSE_DISTANCES, CLOSE_SPEEDS);
        closeHoodLerp = new LinearInterpolation(CLOSE_DISTANCES, CLOSE_HOODS);
        farSpeedLerp = new LinearInterpolation(FAR_DISTANCES, FAR_SPEEDS);
        farHoodLerp = new LinearInterpolation(FAR_DISTANCES, FAR_HOODS);
    }

    // =========================================================================
    // RESULT CLASS
    // =========================================================================

    public static class ShotParameters {
        public final double hoodAngle;      // radians
        public final double turretAngle;    // radians (robot frame)
        public final double flywheelTicks;  // motor ticks/s
        public final double vrr;            // radial robot velocity (in/s)
        public final double vrt;            // tangential robot velocity (in/s)
        public final double vxNew;          // compensated horizontal velocity (in/s)
        public final double launchAngle;    // radians
        public final double launchVelocity; // in/s
        public final boolean isValid;
        public final String errorMessage;

        public ShotParameters(double hoodAngle, double turretAngle, double flywheelTicks,
                              double vrr, double vrt, double vxNew, double launchAngle,
                              double launchVelocity) {
            this.hoodAngle = hoodAngle;
            this.turretAngle = turretAngle;
            this.flywheelTicks = flywheelTicks;
            this.vrr = vrr;
            this.vrt = vrt;
            this.vxNew = vxNew;
            this.launchAngle = launchAngle;
            this.launchVelocity = launchVelocity;
            this.isValid = true;
            this.errorMessage = null;
        }

        public ShotParameters(double hoodAngle, double turretAngle, double flywheelTicks) {
            this(hoodAngle, turretAngle, flywheelTicks, Double.NaN, Double.NaN, Double.NaN,
                    Double.NaN, Double.NaN);
        }

        public ShotParameters(String error) {
            this.hoodAngle = 0;
            this.turretAngle = 0;
            this.flywheelTicks = 0;
            this.vrr = Double.NaN;
            this.vrt = Double.NaN;
            this.vxNew = Double.NaN;
            this.launchAngle = Double.NaN;
            this.launchVelocity = Double.NaN;
            this.isValid = false;
            this.errorMessage = error;
        }
    }

    // =========================================================================
    // MAIN CALCULATION
    // =========================================================================

    /**
     * Calculate shot parameters with velocity compensation.
     * 
     * @param robotPose   Robot position (x, y, heading)
     * @param robotVel    Robot velocity (vx, vy in in/s, omega in rad/s)
     * @param goalPose    Goal Position (x,y)
     * @param close       True for close zone, false for far zone
     */
    public static ShotParameters calculate(Pose robotPose, Pose robotVel,
                                           Pose goalPose, boolean close) {

        // -----------------------------------------------------------------
        // 1. Calculate shooter position
        // -----------------------------------------------------------------
        double cosH = Math.cos(robotPose.getHeading());
        double sinH = Math.sin(robotPose.getHeading());
        
        double shooterX = robotPose.getX() + SHOOTER_OFFSET_X * cosH - SHOOTER_OFFSET_Y * sinH;
        double shooterY = robotPose.getY() + SHOOTER_OFFSET_X * sinH + SHOOTER_OFFSET_Y * cosH;

        // -----------------------------------------------------------------
        // 2. Calculate distance and angle to goal
        // -----------------------------------------------------------------


        double dx = goalPose.getX() - shooterX;
        double dy = goalPose.getY() - shooterY;
        double distance = Math.sqrt(dx * dx + dy * dy);
        double angleToGoal = Math.atan2(dy, dx);

        if (distance < 1.0) {
            return new ShotParameters("Too close to goal");
        }

        // -----------------------------------------------------------------
        // 3. Get base values from calibration tables
        // -----------------------------------------------------------------
        LinearInterpolation speedLerp = close ? closeSpeedLerp : farSpeedLerp;
        LinearInterpolation hoodLerp = close ? closeHoodLerp : farHoodLerp;
        double[] distances = close ? CLOSE_DISTANCES : FAR_DISTANCES;

        // Clamp distance to calibration range
        double minDist = distances[0];
        double maxDist = distances[0];
        for (double value : distances) {
            if (value < minDist) {
                minDist = value;
            }
            if (value > maxDist) {
                maxDist = value;
            }
        }
        double clampedDist = Math.max(minDist, Math.min(maxDist, distance));

        double baseFlywheelTicks = speedLerp.interpolate(clampedDist);
        double baseHoodAngle = hoodLerp.interpolate(clampedDist);

        // Calculate approximate launch angle and velocity from calibration
        double launchAngle = hoodAngleToLaunchAngle(baseHoodAngle);
        double baseLaunchVelocity = estimateLaunchVelocity(baseFlywheelTicks );
        double baseVx = baseLaunchVelocity * Math.cos(launchAngle);
        double baseVy = baseLaunchVelocity * Math.sin(launchAngle);
        double inferredGoalHeight = inferGoalHeight(distance, baseLaunchVelocity, launchAngle);

        // -----------------------------------------------------------------
        // 4. Velocity Compensation
        // -----------------------------------------------------------------
        double vRobotX = robotVel.getX();
        double vRobotY = robotVel.getY();
        double robotSpeed = Math.sqrt(vRobotX * vRobotX + vRobotY * vRobotY);

        double turretOffset = 0;
        double flywheelTicks = baseFlywheelTicks;
        double hoodAngle = baseHoodAngle;
        double vrr = 0;
        double vrt = 0;
        double vxNew = baseVx;
        double compensatedLaunchAngle = launchAngle;
        double compensatedLaunchVelocity = baseLaunchVelocity;

        if (robotSpeed > 1.0) {
            // Decompose robot velocity into radial and tangential
            double robotVelAngle = Math.atan2(vRobotY, vRobotX);
            double deltaAngle = robotVelAngle - angleToGoal;

            // Radial velocity (positive = moving toward goal)
            vrr = -Math.cos(deltaAngle) * robotSpeed;
            // Tangential velocity
            vrt = Math.sin(deltaAngle) * robotSpeed;

            if (baseVx < 1.0) {
                return new ShotParameters("Invalid base horizontal velocity");
            }

            // Estimate flight time from base shot
            double flightTime = distance / baseVx;

            // Compensated horizontal velocity
            double Vx_comp = baseVx + vrr;
            if (Vx_comp <= 0) {
                return new ShotParameters("Robot moving toward goal too fast");
            }

            // New horizontal velocity (with tangential)
            vxNew = Math.sqrt(Vx_comp * Vx_comp + vrt * vrt);

            // New launch angle (keep vertical component the same)
            double newLaunchAngle = Math.atan2(baseVy, vxNew);
            double unclampedHoodAngle = Math.toRadians(90) - newLaunchAngle;
            hoodAngle = clamp(unclampedHoodAngle, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE);
            compensatedLaunchAngle = hoodAngleToLaunchAngle(hoodAngle);

            // Recompute required launch speed using updated x distance
            double xNew = vxNew * flightTime;
            double newLaunchVelocity = solveLaunchVelocity(xNew, inferredGoalHeight,
                    compensatedLaunchAngle);
            if (Double.isNaN(newLaunchVelocity) || newLaunchVelocity <= 0) {
                return new ShotParameters("Invalid compensated launch velocity");
            }

            compensatedLaunchVelocity = newLaunchVelocity;
            flywheelTicks = launchVelocityToTicks(newLaunchVelocity);

            // Turret offset to cancel tangential velocity
            turretOffset = -Math.atan2(vrt, Vx_comp);
        }

        // -----------------------------------------------------------------
        // 5. Apply compensation
        // -----------------------------------------------------------------

        // Turret angle
        double turretAngle = angleToGoal + turretOffset - robotPose.getHeading();
        turretAngle = MathHelpers.wrapAngleRadians(turretAngle);

        return new ShotParameters(hoodAngle, turretAngle, flywheelTicks, vrr, vrt, vxNew,
                compensatedLaunchAngle, compensatedLaunchVelocity);
    }

    /**
     * Calculate without velocity compensation.
     */
    public static ShotParameters calculateStationary(Pose robotPose,
                                                      Pose goalPose, boolean close) {
        return calculate(robotPose, new Pose(0, 0, 0), goalPose, close);
    }

    // =========================================================================
    // HELPERS
    // =========================================================================

// =========================================================================
// VELOCITY CALIBRATION DATA (from range tests)
// Maps motor ticks/s -> launch velocity in/s
// =========================================================================

    private static final double[] CALIBRATION_TPS = {1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700};

    // No-drag velocities (v₀ from spreadsheet column G)
    private static final double[] CALIBRATION_VELOCITIES = {178.48, 194.73, 212.03, 224.88, 235.74, 245.10, 259.50, 273.62};

// Drag-corrected velocities (v₃ from spreadsheet column P)
// private static final double[] CALIBRATION_VELOCITIES = {185.05, 203.04, 222.53, 237.25, 249.87, 260.88, 278.09, 295.96};

    private static LinearInterpolation velocityLerp = new LinearInterpolation(CALIBRATION_TPS, CALIBRATION_VELOCITIES);

    /**
     * Estimate launch velocity from flywheel ticks using calibration data.
     */
    private static double estimateLaunchVelocity(double ticks) {
        double clampedTicks = Math.max(1000, Math.min(1700, ticks));
        return velocityLerp.interpolate(clampedTicks);
    }


    private static LinearInterpolation ticksLerp = new LinearInterpolation(CALIBRATION_VELOCITIES, CALIBRATION_TPS);

    /**
     * Convert launch velocity to flywheel ticks using calibration data.
     */
    private static double launchVelocityToTicks(double launchVelocity) {
        double clampedVelocity = Math.max(178.48, Math.min(273.62, launchVelocity));
        return ticksLerp.interpolate(clampedVelocity);
    }



    private static double inferGoalHeight(double distance, double launchVelocity,
                                          double launchAngle) {
        double cos = Math.cos(launchAngle);
        if (Math.abs(cos) < 1e-6 || launchVelocity <= 0) {
            return 0;
        }
        double term = g * distance * distance;
        return distance * Math.tan(launchAngle) - term / (2 * launchVelocity
                * launchVelocity * cos * cos);
    }

    private static double solveLaunchVelocity(double distance, double height,
                                              double launchAngle) {
        double cos = Math.cos(launchAngle);
        if (Math.abs(cos) < 1e-6) {
            return Double.NaN;
        }
        double denom = distance * Math.tan(launchAngle) - height;
        if (denom <= 0) {
            return Double.NaN;
        }
        return Math.sqrt(g * distance * distance / (2 * cos * cos * denom));
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static double getMinHoodAngle() { return MIN_HOOD_ANGLE; }
    public static double getMaxHoodAngle() { return MAX_HOOD_ANGLE; }
    
    public static double hoodAngleToLaunchAngle(double hoodAngle) {
        return Math.toRadians(90) - hoodAngle;
    }
}
