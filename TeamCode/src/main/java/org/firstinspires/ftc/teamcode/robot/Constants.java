package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class Constants {
    // SHOOTER CONSTANTS
    public static double STRAIGHT_TRAVEL_DISTANCE = 2; // INCHES, TODO: Calculate this
    public static double STRAIGHT_TRAVEL_TIME = 0.5; // SECONDS TODO: Calculate this

    public static double MIN_HOOD_ANGLE = Math.toRadians(40);
    public static double MAX_HOOD_ANGLE = Math.toRadians(80);
    public static double FLYWHEEL_FIRST_CONTACT_ANGLE = Math.toRadians(-18.241);

    public static double MAX_LAUNCH_ANGLE = Math.toRadians(50);
    public static double MIN_LAUNCH_ANGLE = Math.toRadians(10);

    // MIN_HOOD_ANGLE -> MAX_LAUNCH_ANGLE
    // MAX_HOOD_ANGLE -> MIN_LAUNCH_ANGLE
    // 40 arc -> 50 angle
    // 80 arc -> 10 angle

    public static double FLYWHEEL_RADIUS = 1.41732; // INCHES
    public static double BALL_RADIUS = 2.5; // INCHES
    public static double TOTAL_RADIUS = FLYWHEEL_RADIUS + BALL_RADIUS;

    //TURRET CONSTANTS
    public static double MIN_TURRET_ANGLE = Math.toRadians(-180);
    public static double MAX_TURRET_ANGLE = Math.toRadians(180);

    public static double MIN_TURRET_TICKS = -1000;
    public static double MAX_TURRET_TICKS = 1000;

    //TURRET_MIN_ANGLE -> TURRET_MIN_TICKS
    //TURRET_MAX_ANGLE -> TURRET_MAX_TICKS

    public static Pose BLUE_GOAL_POSE = new Pose(10, 134, Math.toRadians(0));
    public static Pose RED_GOAL_POSE = new Pose(134, 134, Math.toRadians(0));



    public static double GOAL_HEIGHT = 38.75; // INCHES
    public static double GRAVITY = 386.0885; // INCHES PER SECOND SQUARED

    public static Robot robot;
}
