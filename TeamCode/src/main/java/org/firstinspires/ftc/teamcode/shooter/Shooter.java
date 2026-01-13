package org.firstinspires.ftc.teamcode.shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
        gateServo = hardwareMap.get(Servo.class, "Gate Servo");

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

    public Command adjustShootingParameters(boolean close) {
        return Commands.instant(() -> updateShootingParameters(close));
    }

    public boolean readyToShoot() {
        return Math.abs(flywheel.getTargetInTicks() - flywheel.getCurrentVelocityInTicks()) < flywheelToleranceTicks &&
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

    public void update() {
        flywheel.update();
        turret.update();
    }

    private void setOpenGatePosition() {
        gateServo.setPosition(openGatePosition);
    }

    private void setCloseGatePosition() {
        gateServo.setPosition(closedGatePosition);
    }

    // TODO: gateServo.getPosition() returns the last commanded position, not actual position.
    // Consider using a timer-based approach or external sensor to confirm servo has reached target.
    private boolean isGateOpen() {
        return Math.abs(gateServo.getPosition() - openGatePosition) < 0.01;
    }

    private boolean isGateClosed() {
        return Math.abs(gateServo.getPosition() - closedGatePosition) < 0.01;
    }

    public Command openGate() {
       return Command.build()
           .setExecute(this::setOpenGatePosition)
           .setDone(this::isGateOpen);
    }

    public Command closeGate() {
        return Command.build()
            .setExecute(this::setCloseGatePosition)
            .setDone(this::isGateClosed);
    }

}
