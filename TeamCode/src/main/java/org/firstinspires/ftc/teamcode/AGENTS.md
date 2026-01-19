# AGENTS.md - TeamCode

Guidance for AI coding agents working in this FTC TeamCode directory.

## Quick Context

- Repository root: `MOEbo-Sapiens-Decode/`
- Primary code: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`
- Java 1.8 (FTC SDK 11.0.0). Follow Android/FTC conventions.
- TeleOp/Auto tests are run on hardware; no unit tests in repo currently.

## Build, Lint, Test Commands

Run from repository root (`MOEbo-Sapiens-Decode/`):

```bash
./gradlew build                 # Build all modules
./gradlew assembleDebug         # Build debug APK
./gradlew :TeamCode:assembleDebug
./gradlew clean                 # Clean artifacts
```

Optional lint (Android lint is available, but not configured with custom rules):

```bash
./gradlew lint
./gradlew :TeamCode:lint
```

Tests:

- No unit tests exist in this repo. Validation happens on FTC hardware.
- If tests are added later, run a single JUnit test with:

```bash
./gradlew :TeamCode:testDebugUnitTest --tests "com.example.ClassName#methodName"
```

## Directory Structure

```
teamcode/
├── drivetrains/     # Drivetrain implementations (Swerve, Mecanum)
├── opmodes/         # OpModes (TeleOp and Autonomous programs)
├── pedroPathing/    # Pedro Pathing configuration
├── robot/           # Robot class, Constants, Intake, Turret
├── shooter/         # Shooter subsystem (Hood, Flywheel, Turret)
├── states/          # State machine implementations
├── util/            # Utility classes (MathHelpers, Vector2D)
└── vision/          # Vision processing (Limelight, pipelines)
```

## Code Style

Follow the Google Java Style Guide with these local notes:

- Java 1.8 only.
- 4-space indentation; no tabs.
- Opening braces on the same line as declarations.
- Limit lines to 100 characters when practical.
- Avoid wildcard imports.

### Import Order

1. Static imports first
2. Third-party imports (`com.*`)
3. Project imports (`org.firstinspires.ftc.*`)

```java
import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
```

### Naming Conventions

| Element | Convention | Example |
|---------|------------|---------|
| Classes | PascalCase | `Robot`, `LimelightManager` |
| Interfaces | PascalCase | `Drivetrain`, `State` |
| Methods | camelCase | `updateDrive()` |
| Variables | camelCase | `currentState` |
| Constants | UPPER_SNAKE_CASE | `BLUE_GOAL_POSE` |
| Enum values | UPPER_SNAKE_CASE | `INTAKING`, `SHOOTING` |

### Types and Access

- Prefer `private` fields and expose accessors when needed.
- Use `final` for references that should not change.
- Avoid nullable returns; use defaults when possible.

## OpMode Pattern

```java
@TeleOp
public class ExampleOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware here

        waitForStart();
        while (opModeIsActive()) {
            // Main loop
            telemetry.update();
        }
    }
}
```

- Use `@TeleOp` for driver-controlled, `@Autonomous` for autonomous.
- Extend `LinearOpMode`, override `runOpMode()`.
- Call `waitForStart()` before the main loop.

## Architecture Patterns

1. Interface-based abstractions: `Drivetrain`, `State`, `VisionPipeline`
2. Supplier pattern: `DrivetrainSupplier`, `StateSupplier`, `VisionPipelineSupplier`
3. Enum-based selection: `Drivetrains`, `States`, `Pipelines` enums
4. Centralized `Robot.java` manages subsystems and the command loop

## PedroPathing + Swerve Integration

- `pedroPathing/PedroConstants` builds a `Follower` via `FollowerBuilder`.
- Swerve drivetrain is configured with `SwerveConstants` and four `CoaxialPod` instances.
- TeleOp `drivetrains/Swerve` wraps PedroPathing and calls
  `((com.pedropathing.ftc.drivetrains.Swerve) follower.getDrivetrain()).arcadeDrive(...)`.
- Follow that pattern when adding new drivetrain behaviors or tuning constants.

## Hardware Names

- Motors: `left_drive`, `right_drive`, `turret_motor`
- Servos: `"Gate Servo"` (descriptive with spaces)
- Sensors: `limelight`, `imu`

## Constants

Use `@Config` (or `@Configurable` where applicable) for dashboard-tunable values:

```java
@Config
public class Constants {
    public static Pose BLUE_GOAL_POSE = new Pose(10, 134, Math.toRadians(0));
}
```

## Error Handling

- Null check before accessing potentially uninitialized objects.
- Return sensible defaults instead of null.
- Use ternary for null-safe returns:

```java
return currentPipeline != null ? currentPipeline.getResult() : new VisionResult();
```

## Creating New Components

### New Subsystem

1. Create class in `robot/` or `shooter/`.
2. Accept `HardwareMap` in constructor.
3. Optionally accept `Telemetry` for debug.
4. Implement `activate()`, `deactivate()`, `toggle()` if applicable.

### New State

1. Create class in `states/` implementing `State`.
2. Implement `initialize(Robot, State)`, `execute(Robot)`, `name()`.
3. Register in `States` enum.

### New Vision Pipeline

1. Create class in `vision/pipelines/` implementing `VisionPipeline`.
2. Implement `initialize(Limelight3A)`, `process(Robot)`, `getResult()`, `name()`.
3. Register in `Pipelines` enum.

## Key Dependencies

- **FTC SDK 11.0.0**: Core framework
- **PedroPathing**: Path following (`ftc:2.1.0-SNAPSHOT`, `ivy:0.0.1-SNAPSHOT`)
- **homeostasis-FTC**: Control systems (`1.0.8`)
- **smile-interpolation**: Math interpolation (`2.6.0`)

## Comments

- Use `//TODO:` with context: `//TODO: adjust as needed`, `//TODO: Tune`.
- Avoid excessive comments for self-explanatory code.

## Pod Tuning Utilities

- `pedroPathing/Tuning` -> `Pod Tuning` folder includes `PodPDFAutoTuner` and
  `PodEncoderMinMaxCalibrator`.
- `PodPDFAutoTuner` runs all four pods in parallel, estimates P/D and F (static friction), and
  prints results to `telemetry` and `telemetryM`; no files are written.
- Ensure `PodPDFAutoTuner` arrays for analog min/max and servo directions match
  `pedroPathing/PedroConstants` for accurate angle scaling.
