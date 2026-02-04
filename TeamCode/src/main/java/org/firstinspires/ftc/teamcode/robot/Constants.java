package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.shooter.Turret;

@Config
public class Constants {
    public static Pose BLUE_GOAL_POSE = new Pose(9, 135, Math.toRadians(0));
    //public static Pose RED_GOAL_POSE = new Pose(134, 134, Math.toRadians(0));
    public enum Color {
        BLUE,
        RED,
        AUDIENCE
    }

    public static Color color = Color.BLUE;

    public static Robot robot;
    public static Pose lastPose = null;
    public static int lastTurretTicks = 0;

    public static void reset() {
        robot = null;
        lastPose = null;
        lastTurretTicks = 0;
        Turret.turretOffset = 0;
    }

    public static boolean lastOpModeWasAuto = false;

    /** Threshold for motor power caching - if power change is less than this, skip setPower call */
    public static double MOTOR_POWER_THRESHOLD = 0.01;
}
