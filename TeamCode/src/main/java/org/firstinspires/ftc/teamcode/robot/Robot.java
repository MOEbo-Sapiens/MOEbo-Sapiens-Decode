package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.states.InitialState;
import org.firstinspires.ftc.teamcode.vision.LimelightManager;
import org.firstinspires.ftc.teamcode.vision.pipelines.ArtifactColorPipeline;
import org.firstinspires.ftc.teamcode.vision.pipelines.LocalizationAprilTagPipeline;
import org.firstinspires.ftc.teamcode.vision.pipelines.ObeliskAprilTagPipeline;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

public class Robot {

    private State currentState;
    private final HardwareMap hardwareMap;

    private final Map<State, Supplier<State>> instanceStateMap = new HashMap<>();
    private final Map<Drivetrain.DriveMode, Drivetrain> driveMap = new HashMap<>();
    private Drivetrain.DriveMode driveMode = Drivetrain.DriveMode.SWERVE;
    private Drivetrain currentDrive;
    Gamepad gamepad1;
    Gamepad gamepad2;
    private LimelightManager vision;
    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.hardwareMap = hardwareMap;

        switchState(State.INITIAL);
    }

    private void initializeMaps() {
        driveMap.put(Drivetrain.DriveMode.MECANUM, new Mecanum(hardwareMap));
        driveMap.put(Drivetrain.DriveMode.SWERVE, new Swerve(hardwareMap));
        driveMap.put(Drivetrain.DriveMode.SWERVE_ANGLE, new SwerveAngle(hardwareMap));
        driveMap.put(Drivetrain.DriveMode.MECANUM_ANGLE, new MecanumAngle(hardwareMap));

        instanceStateMap.put(State.StateNames.SHOOTING, () -> new InitialState(joystick));
        instanceStateMap.put(State.StateNames.INTAKING, () -> new InitialState(joystick));

        vision = new LimelightManager(hardwareMap);
        vision.registerPipeline(new ArtifactColorPipeline(0, vision));
        vision.registerPipeline(new ObeliskAprilTagPipeline(1, vision));
        vision.registerPipeline(new LocalizationAprilTagPipeline(2, vision));
    }

    public void execute(Telemetry telemetry) {
        telemetry.addData("State:", getCurrentState().name());
        currentState.execute(this, telemetry);
        vision.updateAll();
        telemetry.update();
    }
    public LimelightManager getVision() {
        return vision;
    }

    public State getCurrentState() {
        if (currentState != null) {
            return currentState.getState();
        } else {
            return State.INVALID;
        }
    }

    public void switchState(State newState) {
        IRobot prevState = currentState;
        currentState = Objects.requireNonNull(instanceStateMap.get(newState)).get();
        currentState.initialize(this, prevState);
    }
    public void updateDrive(Telemetry telemetry, double speed, double rotSpeed) {
        currentDrive.update(telemetry, joystick, speed, rotSpeed);
    }

    public void switchDriveMode(IDrive.DriveMode newMode) {
        if (driveMap.containsKey(newMode)) {
            currentDriveMode = newMode;
            currentDrive = driveMap.get(newMode);
        }
    }

    public String getCurrentDriveModeName() {
        return currentDrive.getDriveModeName();
    }


}