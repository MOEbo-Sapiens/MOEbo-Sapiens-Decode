package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.groups.Groups.parallel;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivetrains.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.states.State;
import org.firstinspires.ftc.teamcode.vision.LimelightManager;
import org.firstinspires.ftc.teamcode.vision.Pipelines;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline;
import org.firstinspires.ftc.teamcode.vision.VisionResult;

public class Robot {

    private State currentState;

    Intake intake;
    Shooter shooter;

    Follower follower;

    private Drivetrain drivetrain;
    private LimelightManager limelightManager;

    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;
    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        this.telemetry = telemetry;

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, Follower);

        follower = PedroConstants.createFollower(hardwareMap);

        setDrivetrain(Drivetrains.SWERVE);
        setState(States.SHOOTING);

        limelightManager = new LimelightManager(hardwareMap, telemetry);
        setPipeline(Pipelines.APRIL_TAG);
    }

    public void init() {
        schedule(
            infinite(this::updateDrive),
            infinite(this::limelightProcess),
            infinite(this::executeCurrentState),
            infinite(this::updateTelemetry)
        );
    }

    public void updateTelemetry() {
        telemetry.addData("Drivetrain:", drivetrainName());
    }

    public void limelightProcess() {
        telemetry.addData("State:", getCurrentState().name());
        telemetry.addData("Pipeline:", limelightManager.getCurrentPipelineName());
        limelightManager.process(this);
    }

    public void executeCurrentState() {
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
        currentState = newState.build(telemetry);
        currentState.initialize(this, prevState);
    }

    public void setDrivetrain(Drivetrains drivetrain) {
       this.drivetrain = drivetrain.build(follower, telemetry);
    }

    public void updateDrive() {
        drivetrain.update(gamepad1);
    }

    public void updateDrive(double speed, double rotSpeed) {
        drivetrain.update(gamepad1, speed, rotSpeed);
    }

    public String drivetrainName() {
        return drivetrain.name();
    }

    public void setPipeline(Pipelines pipeline) {
        limelightManager.setPipeline(pipeline);
    }

    public void cycleNextPipeline() {
        limelightManager.cycleNext();
    }

    public void cyclePreviousPipeline() {
        limelightManager.cyclePrevious();
    }

    public Pipelines getCurrentPipeline() {
        return limelightManager.getCurrentPipelineEnum();
    }

    public VisionPipeline getVisionPipeline() {
        return limelightManager.getCurrentPipeline();
    }

    public LimelightManager getLimelightManager() {
        return limelightManager;
    }

    public VisionResult getVisionResult() {
        return limelightManager.getResult();
    }
}
