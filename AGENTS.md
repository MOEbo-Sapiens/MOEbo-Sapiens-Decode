# AGENTS.md - MOEbo-Sapiens-Decode

Guidance for agentic coding tools working in this repository.

## Scope and Precedence

- This file applies to the entire repo.
- There is a more specific `AGENTS.md` under
  `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.
- When editing files inside that subtree, follow the deeper instructions.
- No Cursor/Copilot rules were found in `.cursor/rules/`, `.cursorrules`, or
  `.github/copilot-instructions.md`.

## Quick Context

- Repo root: `MOEbo-Sapiens-Decode/`
- Android/FTC SDK project with Gradle build.
- Primary robot code: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`
- Controller app: `FtcRobotController/`
- Java 1.8 (FTC SDK 11.0.0); Android Studio Ladybug (2024.2+) recommended.

## Build, Lint, Test Commands

Run from repo root (`MOEbo-Sapiens-Decode/`).

### Build

```bash
./gradlew build
./gradlew assembleDebug
./gradlew :TeamCode:assembleDebug
./gradlew :FtcRobotController:assembleDebug
./gradlew clean
```

### Lint (optional)

```bash
./gradlew lint
./gradlew :TeamCode:lint
./gradlew :FtcRobotController:lint
```

### Tests

- No unit tests are currently present; validation typically happens on hardware.
- If JUnit tests are added later, run a single test with:

```bash
./gradlew :TeamCode:testDebugUnitTest --tests "com.example.ClassName#methodName"
```

## Repository Layout

```
FtcRobotController/    # FTC SDK controller app
TeamCode/              # Team-specific robot code
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
libs/                  # Android libraries and README
```

## Code Style (Java)

Follow Google Java Style with these repo-specific rules:

- Java 1.8 only.
- 4-space indentation; no tabs.
- Opening braces on the same line.
- Keep lines ~100 chars when practical.
- Avoid wildcard imports (`*`).
- Prefer `final` for references that should not change.
- Prefer `private` fields with accessors.

### Import Order

1. Static imports
2. Third-party imports (`com.*`, `android.*`, `androidx.*`)
3. Project imports (`org.firstinspires.ftc.*`)

```java
import static com.pedropathing.ivy.Scheduler.schedule;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
```

## Naming Conventions

- Classes/Interfaces: PascalCase (`Robot`, `LimelightManager`)
- Methods/Variables: camelCase (`updateDrive`, `currentState`)
- Constants: UPPER_SNAKE_CASE (`BLUE_GOAL_POSE`)
- Enums: UPPER_SNAKE_CASE (`INTAKING`, `SHOOTING`)

## Types and Null Safety

- Avoid nullable returns; return defaults or empty objects.
- Null-check hardware or pipeline instances before use.
- Prefer primitive types (`double`, `int`) when possible.
- Use ternary for null-safe returns when concise.

Example:

```java
return currentPipeline != null ? currentPipeline.getResult() : new VisionResult();
```

## OpMode Guidance

- Use `@TeleOp` for driver-controlled, `@Autonomous` for autonomous.
- Prefer `LinearOpMode` unless there is a specific reason to use iterative.
- Call `waitForStart()` before the active loop.
- Keep the main loop responsive; avoid long blocking calls.

```java
@TeleOp
public class ExampleOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware here

        waitForStart();
        while (opModeIsActive()) {
            telemetry.update();
        }
    }
}
```

## Error Handling

- Handle invalid hardware names or missing devices gracefully.
- Guard against uninitialized subsystems before use.
- Prefer safe defaults and short-circuit logic.
- Avoid throwing exceptions from the main loop.

## FTC/Android Conventions

- Use `HardwareMap` access during init (not during loop) when practical.
- Keep telemetry updates concise and throttled if expensive.
- Use SDK logging/telemetry rather than `System.out`.

## Dependencies and SDK Notes

- FTC SDK 11.0.0, Android Gradle Plugin 8.7.0.
- Some code uses PedroPathing and other libraries configured in
  `TeamCode/build.gradle`.
- Hardware tests run on actual robot hardware; CI/unit tests are not standard.

## When Adding Files

- Prefer editing existing classes and patterns over new abstractions.
- Keep new classes in the appropriate package (`drivetrains/`, `robot/`, etc.).
- Update enums/registries when adding new State/Pipeline/Drivetrain types.

## Documentation

- Avoid adding new README files unless asked.
- Inline comments should be minimal and purposeful.
- Use `//TODO:` with context when needed (no excessive notes).
