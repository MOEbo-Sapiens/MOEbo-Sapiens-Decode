package org.firstinspires.ftc.teamcode.shooter.math;

import static org.firstinspires.ftc.teamcode.util.MathHelpers.wrapAngleRadians;

import com.pedropathing.geometry.Pose;

public class ShootingPoseCalculation {

    //backup if twist version doesn't work
//    public static Pose calculatePoseLinear(Pose currentPos, double shootingTimeSeconds, Pose velocity) {
//        double newX = currentPos.getX() + velocity.getX() * shootingTimeSeconds;
//        double newY = currentPos.getY() + velocity.getY() * shootingTimeSeconds;
//        double newHeading = wrapAngleRadians( currentPos.getHeading() + velocity.getHeading() * shootingTimeSeconds; )
//        return new Pose(newX, newY, newHeading);
//    }

    public static Pose calculatePoseTwist(Pose currentPos, double shootingTimeSeconds, Pose velocity) {
        double omega = velocity.getHeading();
        double dTheta = omega * shootingTimeSeconds;

        double cos = Math.cos(currentPos.getHeading());
        double sin = Math.sin(currentPos.getHeading());
        double vxRobot =  velocity.getX() * cos + velocity.getY() * sin;
        double vyRobot = -velocity.getX() * sin + velocity.getY() * cos;

        double dxBody, dyBody;
        if (Math.abs(omega) < 1e-9) {
            dxBody = vxRobot * shootingTimeSeconds;
            dyBody = vyRobot * shootingTimeSeconds;
        } else {
            double sinDTheta = Math.sin(dTheta);
            double cosDTheta = Math.cos(dTheta);
            dxBody = (vxRobot * sinDTheta - vyRobot * (1 - cosDTheta)) / omega;
            dyBody = (vxRobot * (1 - cosDTheta) + vyRobot * sinDTheta) / omega;
        }

        double dxField = dxBody * cos - dyBody * sin;
        double dyField = dxBody * sin + dyBody * cos;

        double newX = currentPos.getX() + dxField;
        double newY = currentPos.getY() + dyField;
        double newHeading = wrapAngleRadians(currentPos.getHeading() + dTheta);

        return new Pose(newX, newY, newHeading);
    }
}
