package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivetrains.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;
import org.firstinspires.ftc.teamcode.states.State;
import org.firstinspires.ftc.teamcode.vision.LimelightManager;
import org.firstinspires.ftc.teamcode.vision.pipelines.ArtifactColorPipeline;
import org.firstinspires.ftc.teamcode.vision.pipelines.LocalizationAprilTagPipeline;
import org.firstinspires.ftc.teamcode.vision.pipelines.ObeliskAprilTagPipeline;

public class Robot {

    private State currentState;
    private final HardwareMap hardwareMap;

    Intake intake;

    Follower follower;

    private Drivetrain drivetrain;
    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;
    private LimelightManager vision;
    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        this.telemetry = telemetry;

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.hardwareMap = hardwareMap;

        follower = PedroConstants.createFollower(hardwareMap);

        setDrivetrain(Drivetrains.SWERVE);
        setState(States.SHOOTING);

        vision = new LimelightManager(hardwareMap);
    }

    public void execute() {
        telemetry.addData("State:", getCurrentState().name());
        currentState.execute(this);
        vision.updateAll();
        Scheduler.execute();
        telemetry.update();
    }

    public LimelightManager getVision() {
        return vision;
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
}