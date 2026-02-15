package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.hardware.ServoEx;

public class TurretServo {
    private ServoEx turret;
    private final Pose redGoalPose = new Pose(127, 129, 0);

    private final double minServo = 0.0;
    private final double maxServo = 1.0;

    public TurretServo(HardwareMap hardwareMap) {
        turret = new ServoEx(hardwareMap, "turret");

        turret.setPosition(0.5);

    }

    public void update(Pose robotPose) {
        double angleToGoal = Math.atan2(
                redGoalPose.getY() - robotPose.getY(),
                redGoalPose.getX() - robotPose.getX()
        );
        double relativeAngle = normalizeAngle(angleToGoal - robotPose.getHeading());

        double servoPos = (relativeAngle + Math.PI) / (2 * Math.PI);
        servoPos = clamp(servoPos, minServo, maxServo);

        turret.setPosition(servoPos);
    }

    private double normalizeAngle(double angle) {
        angle %= (2 * Math.PI);
        if (angle <= -Math.PI) angle += 2 * Math.PI;
        if (angle > Math.PI) angle -= 2 * Math.PI;
        return angle;
    }

    private double clamp(double val, double min, double max) {
        if (val < min) return min;
        if (val > max) return max;
        return val;
    }
}

