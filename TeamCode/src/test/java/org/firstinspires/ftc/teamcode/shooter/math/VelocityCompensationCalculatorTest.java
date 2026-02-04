package org.firstinspires.ftc.teamcode.shooter.math;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.shooter.Shooter;

public class VelocityCompensationCalculatorTest {

    public void compareShots() {
        double offset = 2.42 / Math.sqrt(2);
        Pose robotPose = new Pose(72 - offset, 72 + offset, Math.toRadians(135));
        Pose goalPose = Constants.BLUE_GOAL_POSE;
        boolean close = robotPose.getY() > Shooter.transitionYValue;

        Pose[] velocities = new Pose[] {
                new Pose(0, 0, 0),
                new Pose(5, 5, 0),
                new Pose(5, -5, 0),
                new Pose(-5, 5, 0),
                new Pose(-5, -5, 0),
                new Pose(20, 20, 0),
                new Pose(20, -20, 0),
                new Pose(-20, 20, 0),
                new Pose(-20, -20, 0)
        };

        VelocityCompensationCalculator.ShotParameters base =
                VelocityCompensationCalculator.calculateStationary(robotPose, goalPose, close);

        System.out.println("=== Velocity Compensation Shot Comparison ===");
        System.out.println(String.format("Robot pose: (%.3f, %.3f, 135°)",
                robotPose.getX(), robotPose.getY()));
        System.out.println("Goal pose: " + goalPose);
        System.out.println("Zone: " + (close ? "CLOSE" : "FAR"));
        System.out.println("Base (stationary): " + format(base));
        System.out.println("------------------------------------------------");

        for (Pose velocity : velocities) {
            VelocityCompensationCalculator.ShotParameters params =
                    VelocityCompensationCalculator.calculate(robotPose, velocity, goalPose, close);
            System.out.println(String.format("v=(%.1f, %.1f) -> %s",
                    velocity.getX(), velocity.getY(), format(params)));
        }
    }

    private String format(VelocityCompensationCalculator.ShotParameters params) {
        if (!params.isValid) {
            return "INVALID: " + params.errorMessage;
        }
        return String.format("hood=%.2f°, turret=%.2f°, ticks=%.1f, Vrr=%.1f, Vrt=%.1f, "
                        + "VxNew=%.1f, launchV=%.1f, launchA=%.2f°",
                Math.toDegrees(params.hoodAngle),
                Math.toDegrees(params.turretAngle),
                params.flywheelTicks,
                params.vrr,
                params.vrt,
                params.vxNew,
                params.launchVelocity,
                Math.toDegrees(params.launchAngle));
    }
}
