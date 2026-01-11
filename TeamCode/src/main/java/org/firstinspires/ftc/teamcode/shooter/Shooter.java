package org.firstinspires.ftc.teamcode.shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.shooter.math.ShootingPoseCalculation;

public class Shooter {
    Hood hood;
    Flywheel flywheel;
    Turret turret;

    Follower follower;

    public Shooter(HardwareMap hardwareMap, Follower follower) {
        hood = new Hood(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        turret = new Turret(hardwareMap);

        this.follower = follower;
    }


    public double getTargetHoodAngle() {
        Pose currentPose = follower.getPose();
        Vector translationalVel = follower.getVelocity();
        Pose velocity = new Pose(translationalVel.getXComponent(),
                translationalVel.getYComponent(),
                follower.getAngularVelocity()
        );

        Pose predictedShootingPose = ShootingPoseCalculation.calculatePoseTwist(currentPose, Constants.)
    }


}
