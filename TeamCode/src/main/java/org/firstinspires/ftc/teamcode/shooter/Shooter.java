package org.firstinspires.ftc.teamcode.shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.shooter.math.QuadraticInterpolation;
import org.firstinspires.ftc.teamcode.shooter.math.ShooterSolver;
import org.firstinspires.ftc.teamcode.util.MathHelpers;

import smile.interpolation.Interpolation;
import smile.interpolation.LinearInterpolation;

public class Shooter {
    //max speed is 2180 ticks / second -> correlates with roughly 5 radians per second
    //all to red score

    //FAR ZONE:
    //  Speed: 1760, Servo Pos: 0.64, launch angle: 23.7834780759°, hood angle: 66.2167323833°
    //AT THE TOP OF FAR ZONE: 72, 24


    //CLOSE LERP 1:
    //Speed: 1315, servo pos 0.36, launch angle: 35.2598227124°, hood angle: 54.7401772876°
    // Pose: 48, 96

    // Speed 1260, Servo pos 0.35, launch angle 35.6608931689°, hood angle: 54.3393172902°
    // Pose: 72, 72

    //Speed: 1040, Servo pos: 0.1, launch angle 45.9053785459°, hood angle: 44.0946214541°
    //Pose: 96, 96

    //Speed: 1030, Servo Pos: 0.08, launch angle: 46.7247081929°, hood angle: 43.2752918071°
    //Pose: 102, 102


    public static double transitionYValue = 52;

    public double distance(Pose launchPose, Pose goalPose) {
        return Math.hypot(goalPose.getX() - launchPose.getX(), goalPose.getY() - launchPose.getY());
    }

    private double[] closeDistances = new double[] {
            distance(new Pose(48, 96), Constants.BLUE_GOAL_POSE.mirror()),
            distance(new Pose(72, 72), Constants.BLUE_GOAL_POSE.mirror()),
            distance(new Pose(96, 96), Constants.BLUE_GOAL_POSE.mirror()),
            distance(new Pose(102, 102), Constants.BLUE_GOAL_POSE.mirror()),
    };

    private double[] closeSpeeds = new double[] {
            1315,
            1260,
            1040,
            1030
    };

    //hood angles
    private double[] closeAngles = new double[] {
            Math.toRadians(54.7401772876),
            Math.toRadians(54.3393172902),
            Math.toRadians(44.0946214541),
            Math.toRadians(43.2752918071)
    };

    Interpolation closeFlywheelSpeeds = new QuadraticInterpolation(closeDistances, closeSpeeds);
    Interpolation closeHoodAngles = new LinearInterpolation(closeDistances, closeAngles);

    private double[] farDistances = new double[] {
            distance(new Pose(72, 24), Constants.BLUE_GOAL_POSE.mirror()),
            distance(new Pose(72, 24), Constants.BLUE_GOAL_POSE.mirror()) + 0.001
    };

    private double[] farSpeeds = new double[] {
           1760,
           1760
    };

    //hood angles
    private double[] farAngles = new double[] {
            Math.toRadians(66.2167323833),
            Math.toRadians(66.2167323833)
    };

    Interpolation farFlywheelSpeeds = new LinearInterpolation(farDistances, farSpeeds);
    Interpolation farHoodAngles = new LinearInterpolation(farDistances, farAngles);

    public static int flywheelToleranceTicks = 40; //TODO: adjust as needed
    public static double turretToleranceDegrees = 2; //TODO: adjust as needed
    public static double hoodToleranceDegrees = 2; //TODO: adjust as needed


    public static double openGatePosition = 0.3; //TODO: Tune
    public static double closedGatePosition = 0.13; //TODO: Tune

    Hood hood;
    Flywheel flywheel;
    Turret turret;
    Servo gateServo;


    Follower follower;

    Pose goalPose;

    public Shooter(HardwareMap hardwareMap, Follower follower, Pose goalPose) {
        hood = new Hood(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        turret = new Turret(hardwareMap);
        gateServo = hardwareMap.get(Servo.class, "gate");

        this.goalPose = goalPose;

        this.follower = follower;
    }


//    public void updateShootingSubsystems(boolean close) {
//        Pose currentPose = follower.getPose();
//        Vector translationalVel = follower.getVelocity();
//        Pose velocity = new Pose(translationalVel.getXComponent(),
//                translationalVel.getYComponent(),
//                follower.getAngularVelocity()
//        );
//
//        ShotSolution shotSolution = ShooterSolver.solve(currentPose, velocity, goalPose.getX(), goalPose.getY(), close);
//
//        flywheel.setTargetAngularVelocity(shotSolution.flywheelSpeed);
//        hood.setHoodAngle(shotSolution.hoodAngle);
//        turret.setTurretAngle(shotSolution.turretAngle);
//    }

    public void updateShootingSubsystems(Pose pose, Telemetry telemetry) {
        boolean close = pose.getY() > transitionYValue;

        Interpolation flywheelInterpolation = (close) ? closeFlywheelSpeeds : farFlywheelSpeeds;
        Interpolation hoodAngleInterpolation = (close) ? closeHoodAngles : farHoodAngles;

        double flywheelSpeed = flywheelInterpolation.interpolate(distance(pose, goalPose));
        double hoodAngle = hoodAngleInterpolation.interpolate(distance(pose, goalPose));
        double turretAngle = getTargetTurretAngle(pose);

        telemetry.addData("flywheelSpeed", flywheelSpeed);
        telemetry.addData("hoodAngle", Math.toDegrees(hoodAngle));
        telemetry.addData("Turret Angle", Math.toDegrees(turretAngle));
        telemetry.addData("Goal Pose", goalPose);

        flywheel.setTargetAngularVelocity(flywheelSpeed);
        hood.setHoodAngle(hoodAngle);
        turret.setTurretAngle(turretAngle);
    }

    public double getTargetTurretAngle(Pose pose) {
        double targetAngle = Math.atan2(
                goalPose.getY() - pose.getY(),
                goalPose.getX() - pose.getX()
        );

        targetAngle -= pose.getHeading();

        targetAngle = MathHelpers.wrapAngleRadians(targetAngle);

        return targetAngle;
    }



    public boolean readyToShoot() {
        return Math.abs(flywheel.getTargetAngularVelocity() - flywheel.getCurrentAngularVel()) < flywheelToleranceTicks &&
                Math.abs(turret.getTargetAngle() - turret.getCurrentAngle())  < Math.toRadians(turretToleranceDegrees) &&
                Math.abs(hood.getTargetHoodAngle() - hood.getCurrentHoodAngle()) < Math.toRadians(hoodToleranceDegrees);
    }

    public void activate() {
        flywheel.activate();
        turret.activate();
    }

    public void intakingPos() {
        flywheel.deactivate();
        turret.setTurretAngle(Math.toRadians(0));
        hood.setHoodAngle(ShooterSolver.getMinHoodAngle());
    }

    public void deactivate() {
        flywheel.deactivate();
        turret.deactivate();
    }

    public void toggle() {
        flywheel.toggle();
        turret.toggle();
    }

    public void update(Telemetry telemetry) {
        flywheel.update();
        turret.update(telemetry);
    }

    public void setOpenGatePosition() {
        gateServo.setPosition(openGatePosition);
    }

    public void setCloseGatePosition() {
        gateServo.setPosition(closedGatePosition);
    }

    public double getTurretAngle() {
        return turret.getCurrentAngle();
    }

    public double getFlywheelAngularVelocity() {
        return flywheel.getCurrentAngularVel();
    }

    public double getHoodAngle() {
        return hood.getCurrentHoodAngle();
    }
}
