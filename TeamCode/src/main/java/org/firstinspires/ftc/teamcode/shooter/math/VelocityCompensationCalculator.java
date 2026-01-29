package org.firstinspires.ftc.teamcode.shooter.math;

import static org.firstinspires.ftc.teamcode.shooter.Shooter.distance;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.robot.Constants;

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
    private static final double[] CLOSE_DISTANCES = new double[] {
            distance(new Pose(48, 96), Constants.BLUE_GOAL_POSE.mirror()),
            distance(new Pose(72, 72), Constants.BLUE_GOAL_POSE.mirror()),
            distance(new Pose(88, 85), Constants.BLUE_GOAL_POSE.mirror()),
            distance(new Pose(90, 90), Constants.BLUE_GOAL_POSE.mirror()),
            distance(new Pose(96, 96), Constants.BLUE_GOAL_POSE.mirror()),
            distance(new Pose(102, 102), Constants.BLUE_GOAL_POSE.mirror()),
    };
    private static final double[] CLOSE_SPEEDS = {1315, 1220, 1120, 1078, 1034, 1006};
    private static final double[] CLOSE_HOODS = {
        Math.toRadians(54.74), Math.toRadians(42.05), Math.toRadians(40.0),
        Math.toRadians(40.0), Math.toRadians(40.0), Math.toRadians(40.0)
    };

    // Far zone calibration (add your data)
    private static final double[] FAR_DISTANCES = new double[] {
            distance(new Pose(72, 24), Constants.BLUE_GOAL_POSE),
            distance(new Pose(84, 12), Constants.BLUE_GOAL_POSE)
    };
    private static final double[] FAR_SPEEDS = {1712, 1712};
    private static final double[] FAR_HOODS = {Math.toRadians(64.17), Math.toRadians(61.71)};

    // Interpolators
    private static LinearInterpolation closeSpeedLerp;
    private static LinearInterpolation closeHoodLerp;
    private static LinearInterpolation farSpeedLerp;
    private static LinearInterpolation farHoodLerp;

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
        public final boolean isValid;
        public final String errorMessage;

        public ShotParameters(double hoodAngle, double turretAngle, double flywheelTicks) {
            this.hoodAngle = hoodAngle;
            this.turretAngle = turretAngle;
            this.flywheelTicks = flywheelTicks;
            this.isValid = true;
            this.errorMessage = null;
        }

        public ShotParameters(String error) {
            this.hoodAngle = 0;
            this.turretAngle = 0;
            this.flywheelTicks = 0;
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
        double maxDist = distances[distances.length - 1];
        double clampedDist = Math.max(minDist, Math.min(maxDist, distance));

        double baseFlywheelTicks = speedLerp.interpolate(clampedDist);
        double baseHoodAngle = hoodLerp.interpolate(clampedDist);

        // Calculate approximate launch angle and velocity from calibration
        double launchAngle = Math.toRadians(90) - baseHoodAngle;
        double baseLaunchVelocity = estimateLaunchVelocity(baseFlywheelTicks, close);

        // -----------------------------------------------------------------
        // 4. Velocity Compensation
        // -----------------------------------------------------------------
        double vRobotX = robotVel.getX();
        double vRobotY = robotVel.getY();
        double robotSpeed = Math.sqrt(vRobotX * vRobotX + vRobotY * vRobotY);

        double turretOffset = 0;
        double velocityMultiplier = 1.0;

        if (robotSpeed > 1.0) {
            // Decompose robot velocity into radial and tangential
            double robotVelAngle = Math.atan2(vRobotY, vRobotX);
            double deltaAngle = robotVelAngle - angleToGoal;

            // Radial velocity (positive = moving toward goal)
            double Vrr = -Math.cos(deltaAngle) * robotSpeed;
            // Tangential velocity
            double Vrt = Math.sin(deltaAngle) * robotSpeed;

            // Estimate flight time
            double Vx_base = baseLaunchVelocity * Math.cos(launchAngle);
            if (Vx_base < 1.0) Vx_base = 100.0; // Fallback
            double flightTime = distance / Vx_base;

            // Compensated horizontal velocity
            double Vx_comp = Vx_base + Vrr;
            if (Vx_comp <= 0) {
                return new ShotParameters("Robot moving toward goal too fast");
            }

            // New horizontal velocity (with tangential)
            double Vx_new = Math.sqrt(Vx_comp * Vx_comp + Vrt * Vrt);

            // Velocity multiplier (how much faster/slower to shoot)
            velocityMultiplier = Vx_new / Vx_base;

            // Turret offset to cancel tangential velocity
            turretOffset = Math.atan2(Vrt, Vx_comp);
        }

        // -----------------------------------------------------------------
        // 5. Apply compensation
        // -----------------------------------------------------------------

        // Adjusted flywheel speed
        double flywheelTicks = baseFlywheelTicks * velocityMultiplier;

        // Hood angle stays the same (changing it would affect flight time)
        double hoodAngle = baseHoodAngle;

        // Turret angle
        double turretAngle = angleToGoal + turretOffset - robotPose.getHeading();
        turretAngle = wrapAngle(turretAngle);

        return new ShotParameters(hoodAngle, turretAngle, flywheelTicks);
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

    /** //TODO: maybe just lerp on the data? also maybe get rid of drag calcs here or add them elsewhere
     * Estimate launch velocity from flywheel ticks (inverse of your calibration).
     * v = A * omega * R, omega = ticks/28 * 2π * 1.4
     */
    private static double estimateLaunchVelocity(double ticks, boolean close) {
        double A = close ? 0.4155 : 0.3906;
        double R = 1.41732;
        return ticks * A * R * 2 * Math.PI * 1.4 / 28.0;
    }

    private static double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public static double getMinHoodAngle() { return MIN_HOOD_ANGLE; }
    public static double getMaxHoodAngle() { return MAX_HOOD_ANGLE; }
    
    public static double hoodAngleToLaunchAngle(double hoodAngle) {
        return Math.toRadians(90) - hoodAngle;
    }
}
