package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.shooter.Turret;

@Config
public class Constants {
    public static Pose BLUE_GOAL_POSE = new Pose(9, 132.5, Math.toRadians(0));
    //public static Pose RED_GOAL_POSE = new Pose(134, 134, Math.toRadians(0));
    public enum Color {
        BLUE,
        RED,
        AUDIENCE
    }

    public static Color color = Color.BLUE;

    public static Robot robot;
    public static Pose lastPose = null;
    private static int lastTurretTicks = 0;

    public static int getLastTurretTicks() {
        return lastTurretTicks;
    }

    public static void setLastTurretTicks(int turretTicks) {
        if (lastTurretTicks != 0 && turretTicks == 0) {
            getLastTurretTicks(); // method call to method call
        }
       lastTurretTicks = turretTicks;
    }

    public static void reset() {
        robot = null;
        lastPose = null;
        lastTurretTicks = 0;
        Turret.turretOffset = 0;
    }

    public static boolean lastOpModeWasAuto = false;
    public static boolean debugTelemetry = false;

    /** Thresholds for caching wrappers. */
    public static double MOTOR_CACHING_TOLERANCE = 0.05;
    public static double CRSERVO_CACHING_TOLERANCE = 0.05;
    public static double SERVO_CACHING_TOLERANCE = 0.01;

    public static double PROXIMITY_POLL_MS = 100.0;
    public static double PROXIMITY_POLL_MS_FULL = 300.0;
}
