# AGENTS.md - TeamCode

Guidance for AI coding agents working in this FTC TeamCode directory.

## Build Commands

Run from repository root (`MOEbo-Sapiens-Decode/`):

```bash
./gradlew build           # Build the project
./gradlew assembleDebug   # Build debug APK
./gradlew clean           # Clean build artifacts
```

**Note**: No unit tests exist. Testing is done on physical FTC hardware.

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

Follows [Google Java Style Guide](https://google.github.io/styleguide/javaguide.html). Java 1.8 only.

### Naming Conventions

| Element | Convention | Example |
|---------|------------|---------|
| Classes | PascalCase | `Robot`, `LimelightManager` |
| Interfaces | PascalCase | `Drivetrain`, `State` |
| Methods | camelCase | `updateDrive()`, `limelightProcess()` |
| Variables | camelCase | `currentState`, `pipelineIndex` |
| Constants | UPPER_SNAKE_CASE | `BLUE_GOAL_POSE` |
| Enum values | UPPER_SNAKE_CASE | `INTAKING`, `SHOOTING` |

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

### OpMode Pattern

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

- Use `@TeleOp` for driver-controlled, `@Autonomous` for autonomous
- Extend `LinearOpMode`, override `runOpMode()`
- Call `waitForStart()` before main loop

## Architecture Patterns

1. **Interface-based Abstractions**: `Drivetrain`, `State`, `VisionPipeline`
2. **Supplier Pattern**: `DrivetrainSupplier`, `StateSupplier`, `VisionPipelineSupplier`
3. **Enum-based Selection**: `Drivetrains`, `States`, `Pipelines` enums
4. **Centralized Robot Class**: `Robot.java` manages all subsystems

## Hardware Names

- Motors: `left_drive`, `right_drive`, `turret_motor`
- Servos: `"Gate Servo"` (descriptive with spaces)
- Sensors: `limelight`, `imu`

## Constants

Use `@Config` annotation for dashboard-tunable values:

```java
@Config
public class Constants {
    public static Pose BLUE_GOAL_POSE = new Pose(10, 134, Math.toRadians(0));
}
```

## Error Handling

- Null check before accessing potentially uninitialized objects
- Return sensible defaults instead of null
- Use ternary for null-safe returns:

```java
return currentPipeline != null ? currentPipeline.getResult() : new VisionResult();
```

## Creating New Components

### New Subsystem

1. Create class in `robot/` or `shooter/`
2. Accept `HardwareMap` in constructor
3. Optionally accept `Telemetry` for debug
4. Implement `activate()`, `deactivate()`, `toggle()` if applicable

### New State

1. Create class in `states/` implementing `State`
2. Implement `initialize(Robot, State)`, `execute(Robot)`, `name()`
3. Register in `States` enum

### New Vision Pipeline

1. Create class in `vision/pipelines/` implementing `VisionPipeline`
2. Implement `initialize(Limelight3A)`, `process(Robot)`, `getResult()`, `name()`
3. Register in `Pipelines` enum

## Key Dependencies

- **FTC SDK 11.0.0**: Core framework
- **PedroPathing**: Path following (`ftc:2.1.0-SNAPSHOT`, `ivy:0.0.1-SNAPSHOT`)
- **homeostasis-FTC**: Control systems (`1.0.8`)
- **smile-interpolation**: Math interpolation (`2.6.0`)

## Comments

- Use `//TODO:` with context: `//TODO: adjust as needed`, `//TODO: Tune`
- Avoid excessive comments for self-explanatory code
