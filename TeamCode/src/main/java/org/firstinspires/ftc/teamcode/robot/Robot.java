package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivetrains.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.states.State;

import java.util.DoubleSummaryStatistics;
import java.util.List;
import java.util.function.DoubleSupplier;

public class Robot {

    private State currentState;

    Timer timer = new Timer();

    Intake intake;
    Shooter shooter;

    Follower follower;

    private Drivetrain drivetrain;
//    private LimelightManager limelightManager;


    List<LynxModule> allHubs;

    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;
    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, Pose goalPose) {
        this.telemetry = telemetry;

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        intake = new Intake(hardwareMap);

        follower = PedroConstants.createFollower(hardwareMap);
        shooter = new Shooter(hardwareMap, follower, goalPose);

        setDrivetrain(Drivetrains.SWERVE_ANGLE);
        setState(States.NONE);

//        limelightManager = new LimelightManager(hardwareMap, telemetry);
//        setPipeline(Pipelines.APRIL_TAG);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }


    public void init() {
        timer.resetTimer();
        schedule(
                infinite(this::clearCaches),
                infinite(this::updateFollower),
//                infinite(this::limelightProcess),
                infinite(this::executeCurrentState),
                infinite(this::updateShooter),
                infinite(this::updateTelemetry),
                infinite(this::updateLastTurretTicks)
        );
    }


    public Follower getFollower() {
        return follower;
    }

    public void updateLastTurretTicks() {
        Constants.lastTurretTicks = shooter.getTurretTicks();
    }

    public void updateFollower() {
        follower.update();
        Constants.lastPose = follower.getPose();
        telemetry.addData("Updated lastPose", Constants.lastPose);
    }

    public void clearCaches() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        telemetry.addData("Caches cleared", "");
    }

    public Command deactivateShooter() {
        return instant(() -> shooter.deactivate());
    }

    public Command shooterIntakingPos() {
        return instant(() -> shooter.intakingPos());
    }

    public Command activateShooter() {
        return instant(() -> shooter.activate());
    }

    public Command toggleShooter() {
        return instant(() -> shooter.toggle());
    }

    public Command setIntakePower(double power) {
        return instant(() -> intake.setPower(power));
    }

    public Command joysticksToIntakePower(DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
        return infinite(() -> {
            if (rightTrigger.getAsDouble() > 0.05) {
                intake.setPower(rightTrigger.getAsDouble());
            } else if (leftTrigger.getAsDouble() > 0.05) {
                intake.setPower(-leftTrigger.getAsDouble());
            } else {
                intake.setPower(0.4);
            }
        });
    }

    public void updateShooter() {
        shooter.update(telemetry);
    }

    public Command updateShootingSubsystems() {
        return infinite(() -> shooter.updateShootingSubsystems(follower.getPose(), telemetry));
    }

    public boolean readyToShoot() {
        return shooter.readyToShoot();
    }

    public void updateTelemetry() {
        telemetry.addData("Drivetrain:", drivetrainName());
        telemetry.addData("loop time millis", timer.getElapsedTime());
        timer.resetTimer();
        telemetry.update();
    }

//    public void limelightProcess() {
//        telemetry.addData("State:", getCurrentState().name());
//        telemetry.addData("Pipeline:", limelightManager.getCurrentPipelineName());
//        limelightManager.process(this);
//    }

    public void executeCurrentState() {
        telemetry.addData("Current State: ", currentState.name());
        currentState.execute(this);
    }


    public States getCurrentState() {
        if (currentState != null) {
            return States.get(currentState);
        } else {
            return States.NONE;
        }
    }

    public void setState(States newState) {
        State prevState = currentState;
        currentState = newState.build(telemetry, gamepad1, gamepad2);
        currentState.initialize(this, prevState);
    }

    public double getTurretAngleDegrees() {
        return Math.toDegrees(shooter.getTurretAngle());
    }

    public double getHoodAngleDegrees() {
        return Math.toDegrees(shooter.getHoodAngle());
    }

    public double getFlywheelAngularVelocity() {
        return shooter.getFlywheelAngularVelocity();
    }

    public double getTargetFlywheelAngularVelocity() {
        return shooter.getFlywheeelTargetAngularVelocity();
    }

    public void setDrivetrain(Drivetrains drivetrain) {
       this.drivetrain = drivetrain.build(follower, telemetry);
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public void updateDrive() {
        drivetrain.update(gamepad1);
    }

    public void updateDrive(double speed, double rotSpeed) {
        drivetrain.update(gamepad1, speed, rotSpeed);
    }

    public Command openGate() {
        return instant(shooter::setOpenGatePosition);
    }

    public Command shoot() {
        return sequential(
                openGate(),
                setIntakePower(1),
                waitMs(100),
                closeGate(),
                waitMs(100)
        );
    }

    public Command shootMotif() {
        return sequential(
                shoot(),
                waitUntil(() -> readyToShoot()).raceWith(waitMs(1000)),
                shoot(),
                waitUntil(() -> readyToShoot()).raceWith(waitMs(1000)),
                shoot()
        );
    }

    public Command closeGate() {
        return instant(shooter::setCloseGatePosition);
    }

    public String drivetrainName() {
        return drivetrain.name();
    }

//    public void setPipeline(Pipelines pipeline) {
//        limelightManager.setPipeline(pipeline);
//    }
//
//    public void cycleNextPipeline() {
//        limelightManager.cycleNext();
//    }
//
//    public void cyclePreviousPipeline() {
//        limelightManager.cyclePrevious();
//    }
//
//    public Pipelines getCurrentPipeline() {
//        return limelightManager.getCurrentPipelineEnum();
//    }
//
//    public VisionPipeline getVisionPipeline() {
//        return limelightManager.getCurrentPipeline();
//    }
//
//    public LimelightManager getLimelightManager() {
//        return limelightManager;
//    }
//
//    public VisionResult getVisionResult() {
//        return limelightManager.getResult();
//    }
}
