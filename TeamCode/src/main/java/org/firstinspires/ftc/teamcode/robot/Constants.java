package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class Constants {
    static {
        // SHOOTER CONSTANTS
        double STRAIGHT_TRAVEL_DISTANCE = 2; // INCHES, TODO: Calculate this
        double STRAIGHT_TRAVEL_TIME = 0.5; // SECONDS TODO: Calculate this

        double MIN_SHOOTER_ANGLE = Math.toRadians(40);
        double MAX_SHOOTER_ANGLE = Math.toRadians(80);
        double FLYWHEEL_FIRST_CONTACT_ANGLE = Math.toRadians(-18.241);

        double MAX_LAUNCH_ANGLE = Math.toRadians(50);
        double MIN_LAUNCH_ANGLE = Math.toRadians(10);

        // MIN_SHOOTER_ARC -> MAX_SHOOTER_ANGLE
        // MAX_SHOOTER_ARC -> MIN_SHOOTER_ANGLE
        // 40 arc -> 50 angle
        // 80 arc -> 10 angle

        double FLYWHEEL_RADIUS = 1.41732; // INCHES
        double BALL_RADIUS = 2.5; // INCHES
        double TOTAL_RADIUS = FLYWHEEL_RADIUS + BALL_RADIUS;

        Pose BLUE_GOAL_POSE = new Pose(10, 134, Math.toRadians(0));
        Pose RED_GOAL_POSE = new Pose(134, 134, Math.toRadians(0));


        double GOAL_HEIGHT = 38.75; // INCHES
        double GRAVITY = 386.0885; // INCHES PER SECOND SQUARED
    }

    public static Robot robot;
}
