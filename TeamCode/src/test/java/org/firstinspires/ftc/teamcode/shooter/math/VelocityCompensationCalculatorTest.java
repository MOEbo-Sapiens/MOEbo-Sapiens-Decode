package org.firstinspires.ftc.teamcode.shooter.math;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.shooter.Shooter;

public class VelocityCompensationCalculatorTest {

    public void compareShots() {
        double offset = 2.42 / Math.sqrt(2);
        Pose robotPose = new Pose(72 - offset, 72 + offset, Math.toRadians(135));
        Pose goalPose = Constants.BLUE_GOAL_POSE;
        boolean close = robotPose.getY() > Shooter.transitionYValue;

        Vector[] velocities = new Vector[] {
                vectorXY(0, 0),
                vectorXY(5, 5),
                vectorXY(5, -5),
                vectorXY(-5, 5),
                vectorXY(-5, -5),
                vectorXY(20, 20),
                vectorXY(20, -20),
                vectorXY(-20, 20),
                vectorXY(-20, -20)
        };

        VelocityCompensationCalculator.ShotParameters base =
                VelocityCompensationCalculator.calculate(robotPose, new Vector(), goalPose, close);

        System.out.println("=== Velocity Compensation Shot Comparison ===");
        System.out.println(String.format("Robot pose: (%.3f, %.3f, 135°)",
                robotPose.getX(), robotPose.getY()));
        System.out.println("Goal pose: " + goalPose);
        System.out.println("Zone: " + (close ? "CLOSE" : "FAR"));
        System.out.println("Base (stationary): " + format(base));
        System.out.println("------------------------------------------------");

        for (Vector velocity : velocities) {
            VelocityCompensationCalculator.ShotParameters params =
                    VelocityCompensationCalculator.calculate(robotPose, velocity, goalPose, close);
            System.out.println(String.format("v=(%.1f, %.1f) -> %s",
                    velocity.getXComponent(), velocity.getYComponent(), format(params)));
        }
    }

    private Vector vectorXY(double x, double y) {
        Vector vector = new Vector();
        vector.setOrthogonalComponents(x, y);
        return vector;
    }

    private String format(VelocityCompensationCalculator.ShotParameters params) {
        return String.format("hood=%.2f°, turret=%.2f°, ticks=%.1f°",
                Math.toDegrees(params.hoodAngle),
                Math.toDegrees(params.turretAngle),
                params.flywheelTicks);
    }
}
