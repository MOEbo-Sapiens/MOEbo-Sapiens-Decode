package org.firstinspires.ftc.teamcode.shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.Tele;
import org.firstinspires.ftc.teamcode.shooter.math.ShooterSolver;
import org.firstinspires.ftc.teamcode.shooter.math.ShooterSolver.ShotSolution;

public class Shooter {

    public static int flywheelToleranceTicks = 40; //TODO: adjust as needed
    public static double turretToleranceDegrees = 2; //TODO: adjust as needed
    public static double hoodToleranceDegrees = 2; //TODO: adjust as needed


    public static double openGatePosition = 0.8; //TODO: Tune
    public static double closedGatePosition = 0.3; //TODO: Tune

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


    public void updateShootingParameters(boolean close) {
        Pose currentPose = follower.getPose();
        Vector translationalVel = follower.getVelocity();
        Pose velocity = new Pose(translationalVel.getXComponent(),
                translationalVel.getYComponent(),
                follower.getAngularVelocity()
        );

        ShotSolution shotSolution = ShooterSolver.solve(currentPose, velocity, goalPose.getX(), goalPose.getY(), close);

        flywheel.setTargetAngularVelocity(shotSolution.flywheelSpeed);
        hood.setHoodAngle(shotSolution.hoodAngle);
        turret.setTurretAngle(shotSolution.turretAngle);
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
