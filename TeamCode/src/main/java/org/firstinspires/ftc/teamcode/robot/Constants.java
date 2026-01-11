package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class Constants {
    static {
        // SHOOTER CONSTANTS
        double MIN_SHOOTER_ARC = Math.toRadians(40);
        double MAX_SHOOTER_ARC = Math.toRadians(80);

        double MAX_SHOOTER_ANGLE = Math.toRadians(50);
        double MIN_SHOOTER_ANGLE = Math.toRadians(10);

        // 40 arc -> 50 angle
        // 80 arc -> 10 angle

        double CONTACT_ARC_ANGLE = Math.toRadians(-18.241);
        double FLYWHEEL_RADIUS = 1.41732; // INCHES
        double BALL_RADIUS = 2.5; // INCHES
        double TOTAL_RADIUS = FLYWHEEL_RADIUS + BALL_RADIUS;

        Pose BLUE_GOAL_POSE = new Pose(10, 134, Math.toRadians(180));
        Pose RED_GOAL_POSE = new Pose(134, 134, Math.toRadians(180));
        double GOAL_HEIGHT = 47.5; // INCHES
    }

    public static Robot robot;
}
