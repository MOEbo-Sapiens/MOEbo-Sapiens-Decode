package org.firstinspires.ftc.teamcode.shooter.math;

import static org.firstinspires.ftc.teamcode.shooter.Shooter.distance;
import static org.firstinspires.ftc.teamcode.shooter.Shooter.hoodToleranceDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.util.MathHelpers;
import org.firstinspires.ftc.teamcode.util.Vector2D;

import smile.interpolation.LinearInterpolation;

@Config
public class VelocityCompensationCalculator {

    public static double kRad = 4;
    public static double kTan = 7;

    private static final double g = 386.0885; //in / s^2

    /** Shooter offset from robot center (inches) */
    private static final double SHOOTER_OFFSET_X = -2.42;
    private static final double SHOOTER_OFFSET_Y = 0.0;

    /** HOOD ANGLE LIMITS */
    public static final double MIN_HOOD_ANGLE = Math.toRadians(40);
    public static final double MAX_HOOD_ANGLE = Math.toRadians(74);


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

    // Far zone calibration
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

    public static class ShotParameters {
        public final double hoodAngle;      // radians
        public final double turretAngle;    // radians (robot frame)
        public final double flywheelTicks;  // motor ticks/s

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
     * @param robotPose   Robot position (x, y, heading)
     * @param robotVel    Robot velocity (vx, vy in in/s, omega in rad/s)
     * @param goalPose    Goal Position (x,y)
     * @param close       True for close zone, false for far zone
     */
    public static ShotParameters calculate(Pose robotPose, Vector robotVel,
                                           Pose goalPose, boolean close) {

        //shooter offset
        double cosH = Math.cos(robotPose.getHeading());
        double sinH = Math.sin(robotPose.getHeading());
        
        double shooterX = robotPose.getX() + SHOOTER_OFFSET_X * cosH - SHOOTER_OFFSET_Y * sinH;
        double shooterY = robotPose.getY() + SHOOTER_OFFSET_X * sinH + SHOOTER_OFFSET_Y * cosH;


        //distance to goal with offset
        double dx = goalPose.getX() - shooterX;
        double dy = goalPose.getY() - shooterY;
        double distance = Math.sqrt(dx * dx + dy * dy);
        double angleToGoal = Math.atan2(dy, dx);

        Vector shootingVector = new Vector(1, angleToGoal);
        Vector tangentVector = new Vector(1, angleToGoal + Math.PI/2);

        LinearInterpolation speedLerp = close ? closeSpeedLerp : farSpeedLerp;
        LinearInterpolation hoodLerp = close ? closeHoodLerp : farHoodLerp;

        double baseFlywheelTicks = speedLerp.interpolate(distance);
        double baseHoodAngle = hoodLerp.interpolate(distance);

        double launchAngle = hoodAngleToLaunchAngle(baseHoodAngle);
        double baseVx = baseFlywheelTicks * Math.cos(launchAngle);
//        double baseVy = baseFlywheelTicks * Math.sin(launchAngle);


        double tof = distance / baseVx;

        // pose + tof * (v_rad * k_rad * v_tan * k_tan)
        //it doesn't matter that tof has the wrong units because of hte scaling factors k_tan and k_rad

        //get components relative to goal for scaling
        double vRad = robotVel.dot(shootingVector);
        double vTan = robotVel.dot(tangentVector);

        double scaledVRad = vRad * tof * kRad;
        double scaledVTan = vTan * tof * kTan;

        Vector correctionVector = new Vector();
        correctionVector.setOrthogonalComponents(scaledVRad, scaledVTan);

        //convert back to field space
        correctionVector.rotateVector(angleToGoal);

        Pose futurePose = getFuturePose(robotPose, correctionVector);

        shooterX = futurePose.getX() + SHOOTER_OFFSET_X * cosH - SHOOTER_OFFSET_Y * sinH;
        shooterY = futurePose.getY() + SHOOTER_OFFSET_X * sinH + SHOOTER_OFFSET_Y * cosH;

        //distance to goal including velocity and offset
        dx = goalPose.getX() - shooterX;
        dy = goalPose.getY() - shooterY;
        distance = Math.sqrt(dx * dx + dy * dy);
        angleToGoal = Math.atan2(dy, dx);

        double flywheelSpeed = speedLerp.interpolate(distance);
        double hoodAngle = hoodLerp.interpolate(distance);
        double turretAngle = MathHelpers.wrapAngleRadians(angleToGoal - futurePose.getHeading());

        return new ShotParameters(
                hoodAngle,
                turretAngle,
                flywheelSpeed
        );
    }


    public static Pose getFuturePose(Pose currentPose, Vector velocityCompensation) {
        return new Pose(
                currentPose.getX() + velocityCompensation.getXComponent(),
                currentPose.getY() + velocityCompensation.getYComponent(),
                currentPose.getHeading()
        );
    }

    public static double hoodAngleToLaunchAngle(double hoodAngle) {
        return Math.PI/2 - hoodAngle;
    }

    public static double getMinHoodAngle() {
        return MIN_HOOD_ANGLE;
    }

    public static double getMaxHoodAngle() {
        return MAX_HOOD_ANGLE;
    }
}
