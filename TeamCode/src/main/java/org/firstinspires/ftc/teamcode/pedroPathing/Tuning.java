package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.changes;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.stopRobot;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.math.*;
import com.pedropathing.paths.*;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 * This is the Tuning class. It contains a selection menu for various tuning OpModes.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 6/26/2025
 */
@Configurable
@TeleOp(name = "Tuning", group = "Pedro Pathing")
public class Tuning extends SelectableOpMode {
    public static Follower follower;

    @IgnoreConfigurable
    static PoseHistory poseHistory;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @IgnoreConfigurable
    static ArrayList<String> changes = new ArrayList<>();

    public Tuning() {
        super("Select a Tuning OpMode", s -> {
            s.folder("Localization", l -> {
                l.add("Localization Test", LocalizationTest::new);
                l.add("Forward Tuner", ForwardTuner::new);
                l.add("Lateral Tuner", LateralTuner::new);
                l.add("Turn Tuner", TurnTuner::new);
            });
            s.folder("Automatic", a -> {
                a.add("Forward Velocity Tuner", ForwardVelocityTuner::new);
                a.add("Lateral Velocity Tuner", LateralVelocityTuner::new);
                a.add("Forward Zero Power Acceleration Tuner", ForwardZeroPowerAccelerationTuner::new);
                a.add("Lateral Zero Power Acceleration Tuner", LateralZeroPowerAccelerationTuner::new);
            });
            s.folder("Manual", p -> {
                p.add("Translational Tuner", TranslationalTuner::new);
                p.add("Heading Tuner", HeadingTuner::new);
                p.add("Drive Tuner", DriveTuner::new);
                p.add("Line Tuner", Line::new);
                p.add("Centripetal Tuner", CentripetalTuner::new);
            });
            s.folder("Tests", p -> {
                p.add("Line", Line::new);
                p.add("Triangle", Triangle::new);
                p.add("Circle", Circle::new);
            });
            s.folder("Pod Tuning", p -> {
                p.add("Auto Pod PDF Tuner", PodPDFAutoTuner::new);
            });
        });
    }

    @Override
    public void onSelect() {
        if (follower == null) {
            follower = PedroConstants.createFollower(hardwareMap);
            PanelsConfigurables.INSTANCE.refreshClass(this);
        } else {
            follower = PedroConstants.createFollower(hardwareMap);
        }

        follower.setStartingPose(new Pose());

        poseHistory = follower.getPoseHistory();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        Drawing.init();
    }

    @Override
    public void onLog(List<String> lines) {}

    public static void drawOnlyCurrent() {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public static void draw() {
        Drawing.drawDebug(follower);
    }

    /** This creates a full stop of the robot by setting the drive motors to run at 0 power. */
    public static void stopRobot() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0,0,0,true);
    }
}

/**
 * This is the LocalizationTest OpMode. This is basically just a simple mecanum drive attached to a
 * PoseUpdater. The OpMode will print out the robot's pose to telemetry as well as draw the robot.
 * You should use this to check the robot's localization.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 5/6/2024
 */
class LocalizationTest extends OpMode {
    @Override
    public void init() {
        follower.setStartingPose(new Pose(72,72));
    }

    /** This initializes the PoseUpdater, the mecanum drive motors, and the Panels telemetry. */
    @Override
    public void init_loop() {
        telemetryM.debug("This will print your robot's position to telemetry while "
                + "allowing robot control through a basic mecanum drive on gamepad 1.");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();

        telemetryM.debug("drivetrain debug: ", follower.drivetrain.debugString());
        telemetryM.debug("left stick y, ", -gamepad1.left_stick_y);
        telemetryM.debug("left stick x, ", -gamepad1.left_stick_x);
        telemetryM.debug("right stick x, ", -gamepad1.right_stick_x);
        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.debug("total heading:" + follower.getTotalHeading());
        telemetryM.update(telemetry);

        draw();
    }
}

/**
 * This is the ForwardTuner OpMode. This tracks the forward movement of the robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * robot's current distance in ticks to the specified distance in inches. So, to use this, run the
 * tuner, then pull/push the robot to the specified distance using a ruler on the ground. When you're
 * at the end of the distance, record the ticks to inches multiplier. Feel free to run multiple trials
 * and average the results. Then, input the multiplier into the forward ticks to inches in your
 * localizer of choice.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 5/6/2024
 */
class ForwardTuner extends OpMode {
    public static double DISTANCE = 48;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72,72));
        follower.update();
        drawOnlyCurrent();
    }

    /** This initializes the PoseUpdater as well as the Panels telemetry. */
    @Override
    public void init_loop() {
        telemetryM.debug("Pull your robot forward " + DISTANCE + " inches. Your forward ticks to inches will be shown on the telemetry.");
        telemetryM.update(telemetry);
        drawOnlyCurrent();
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated multiplier and draws the robot.
     */
    @Override
    public void loop() {
        follower.update();

        telemetryM.debug("Distance Moved: " + follower.getPose().getX());
        telemetryM.debug("The multiplier will display what your forward ticks to inches should be to scale your current distance to " + DISTANCE + " inches.");
        telemetryM.debug("Multiplier: " + (DISTANCE / ((follower.getPose().getX() - 72) / follower.getPoseTracker().getLocalizer().getForwardMultiplier())));
        telemetryM.update(telemetry);

        draw();
    }
}

/**
 * This is the LateralTuner OpMode. This tracks the strafe movement of the robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * robot's current distance in ticks to the specified distance in inches. So, to use this, run the
 * tuner, then pull/push the robot to the specified distance using a ruler on the ground. When you're
 * at the end of the distance, record the ticks to inches multiplier. Feel free to run multiple trials
 * and average the results. Then, input the multiplier into the strafe ticks to inches in your
 * localizer of choice.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 6/26/2025
 */
class LateralTuner extends OpMode {
    public static double DISTANCE = 48;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72,72));
        follower.update();
        drawOnlyCurrent();
    }

    /** This initializes the PoseUpdater as well as the Panels telemetry. */
    @Override
    public void init_loop() {
        telemetryM.debug("Pull your robot to the right " + DISTANCE + " inches. Your strafe ticks to inches will be shown on the telemetry.");
        telemetryM.update(telemetry);
        drawOnlyCurrent();
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated multiplier and draws the robot.
     */
    @Override
    public void loop() {
        follower.update();

        telemetryM.debug("Distance Moved: " + follower.getPose().getY());
        telemetryM.debug("The multiplier will display what your strafe ticks to inches should be to scale your current distance to " + DISTANCE + " inches.");
        telemetryM.debug("Multiplier: " + (DISTANCE / ((follower.getPose().getY() - 72) / follower.getPoseTracker().getLocalizer().getLateralMultiplier())));
        telemetryM.update(telemetry);

        draw();
    }
}

/**
 * This is the TurnTuner OpMode. This tracks the turning movement of the robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * robot's current angle in ticks to the specified angle in radians. So, to use this, run the
 * tuner, then pull/push the robot to the specified angle using a protractor or lines on the ground.
 * When you're at the end of the angle, record the ticks to inches multiplier. Feel free to run
 * multiple trials and average the results. Then, input the multiplier into the turning ticks to
 * radians in your localizer of choice.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 5/6/2024
 */
class TurnTuner extends OpMode {
    public static double ANGLE = 2 * Math.PI;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72,72));
        follower.update();
        drawOnlyCurrent();
    }

    /** This initializes the PoseUpdater as well as the Panels telemetry. */
    @Override
    public void init_loop() {
        telemetryM.debug("Turn your robot " + ANGLE + " radians. Your turn ticks to inches will be shown on the telemetry.");
        telemetryM.update(telemetry);

        drawOnlyCurrent();
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated multiplier and draws the robot.
     */
    @Override
    public void loop() {
        follower.update();

        telemetryM.debug("Total Angle: " + follower.getTotalHeading());
        telemetryM.debug("The multiplier will display what your turn ticks to inches should be to scale your current angle to " + ANGLE + " radians.");
        telemetryM.debug("Multiplier: " + (ANGLE / (follower.getTotalHeading() / follower.getPoseTracker().getLocalizer().getTurningMultiplier())));
        telemetryM.update(telemetry);

        draw();
    }
}

/**
 * This is the ForwardVelocityTuner autonomous follower OpMode. This runs the robot forwards at max
 * power until it reaches some specified distance. It records the most recent velocities, and on
 * reaching the end of the distance, it averages them and prints out the velocity obtained. It is
 * recommended to run this multiple times on a full battery to get the best results. What this does
 * is, when paired with StrafeVelocityTuner, allows FollowerConstants to create a Vector that
 * empirically represents the direction your mecanum wheels actually prefer to go in, allowing for
 * more accurate following.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/13/2024
 */
class ForwardVelocityTuner extends OpMode {
    private final ArrayList<Double> velocities = new ArrayList<>();
    public static double DISTANCE = 95;
    public static double RECORD_NUMBER = 10;

    private boolean end;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /** This initializes the drive motors as well as the cache of velocities and the Panels telemetry. */
    @Override
    public void init_loop() {
        telemetryM.debug("The robot will run at 1 power until it reaches " + DISTANCE + " inches forward.");
        telemetryM.debug("Make sure you have enough room, since the robot has inertia after cutting power.");
        telemetryM.debug("After running the distance, the robot will cut power from the drivetrain and display the forward velocity.");
        telemetryM.debug("Press B on game pad 1 to stop.");
        telemetryM.debug("pose", follower.getPose());
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    /** This starts the OpMode by setting the drive motors to run forward at full power. */
    @Override
    public void start() {
        for (int i = 0; i < RECORD_NUMBER; i++) {
            velocities.add(0.0);
        }
        follower.startTeleopDrive(true);
        follower.update();
        end = false;
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad 1 will stop the OpMode. This continuously records the RECORD_NUMBER most recent
     * velocities, and when the robot has run forward enough, these last velocities recorded are
     * averaged and printed.
     */
    @Override
    public void loop() {
        if (gamepad1.bWasPressed()) {
            stopRobot();
            requestOpModeStop();
        }

        follower.update();
        draw();


        if (!end) {
            if (Math.abs(follower.getPose().getX()) > (DISTANCE + 72)) {
                end = true;
                stopRobot();
            } else {
                follower.setTeleOpDrive(1,0,0,true);
                //double currentVelocity = Math.abs(follower.getVelocity().getXComponent());
                double currentVelocity = Math.abs(follower.poseTracker.getLocalizer().getVelocity().getX());
                velocities.add(currentVelocity);
                velocities.remove(0);
            }
        } else {
            stopRobot();
            double average = 0;
            for (double velocity : velocities) {
                average += velocity;
            }
            average /= velocities.size();
            telemetryM.debug("Forward Velocity: " + average);
            telemetryM.debug("\n");
            telemetryM.debug("Press A to set the Forward Velocity temporarily (while robot remains on).");

            for (int i = 0; i < velocities.size(); i++) {
                telemetry.addData(String.valueOf(i), velocities.get(i));
            }

            telemetryM.update(telemetry);
            telemetry.update();

            if (gamepad1.aWasPressed()) {
                follower.setXVelocity(average);
                String message = "XMovement: " + average;
                changes.add(message);
            }
        }
    }
}

/**
 * This is the StrafeVelocityTuner autonomous follower OpMode. This runs the robot left at max
 * power until it reaches some specified distance. It records the most recent velocities, and on
 * reaching the end of the distance, it averages them and prints out the velocity obtained. It is
 * recommended to run this multiple times on a full battery to get the best results. What this does
 * is, when paired with ForwardVelocityTuner, allows FollowerConstants to create a Vector that
 * empirically represents the direction your mecanum wheels actually prefer to go in, allowing for
 * more accurate following.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/13/2024
 */
class LateralVelocityTuner extends OpMode {
    private final ArrayList<Double> velocities = new ArrayList<>();

    public static double DISTANCE = 48;
    public static double RECORD_NUMBER = 10;

    private boolean end;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /**
     * This initializes the drive motors as well as the cache of velocities and the Panels
     * telemetryM.
     */
    @Override
    public void init_loop() {
        telemetryM.debug("The robot will run at 1 power until it reaches " + DISTANCE + " inches to the left.");
        telemetryM.debug("Make sure you have enough room, since the robot has inertia after cutting power.");
        telemetryM.debug("After running the distance, the robot will cut power from the drivetrain and display the strafe velocity.");
        telemetryM.debug("Press B on Gamepad 1 to stop.");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    /** This starts the OpMode by setting the drive motors to run left at full power. */
    @Override
    public void start() {
        for (int i = 0; i < RECORD_NUMBER; i++) {
            velocities.add(0.0);
        }
        follower.startTeleopDrive(true);
        follower.update();
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad1 will stop the OpMode. This continuously records the RECORD_NUMBER most recent
     * velocities, and when the robot has run sideways enough, these last velocities recorded are
     * averaged and printed.
     */
    @Override
    public void loop() {
        if (gamepad1.bWasPressed()) {
            stopRobot();
            requestOpModeStop();
        }

        follower.update();
        draw();

        if (!end) {
            if (Math.abs(follower.getPose().getY()) > (DISTANCE + 72)) {
                end = true;
                stopRobot();
            } else {
                follower.setTeleOpDrive(0,1,0,true);
                double currentVelocity = Math.abs(follower.getVelocity().dot(new Vector(1, Math.PI / 2)));
                velocities.add(currentVelocity);
                velocities.remove(0);
            }
        } else {
            stopRobot();
            double average = 0;
            for (double velocity : velocities) {
                average += velocity;
            }
            average /= velocities.size();

            telemetryM.debug("Strafe Velocity: " + average);
            telemetryM.debug("\n");
            telemetryM.debug("Press A to set the Lateral Velocity temporarily (while robot remains on).");
            telemetryM.update(telemetry);

            if (gamepad1.aWasPressed()) {
                follower.setYVelocity(average);
                String message = "YMovement: " + average;
                changes.add(message);
            }
        }
    }
}

/**
 * This is the ForwardZeroPowerAccelerationTuner autonomous follower OpMode. This runs the robot
 * forward until a specified velocity is achieved. Then, the robot cuts power to the motors, setting
 * them to zero power. The deceleration, or negative acceleration, is then measured until the robot
 * stops. The accelerations across the entire time the robot is slowing down is then averaged and
 * that number is then printed. This is used to determine how the robot will decelerate in the
 * forward direction when power is cut, making the estimations used in the calculations for the
 * drive Vector more accurate and giving better braking at the end of Paths.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
class ForwardZeroPowerAccelerationTuner extends OpMode {
    private final ArrayList<Double> accelerations = new ArrayList<>();
    public static double VELOCITY = 77.45;

    private double previousVelocity;
    private long previousTimeNano;

    private boolean stopping;
    private boolean end;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /** This initializes the drive motors as well as the Panels telemetryM. */
    @Override
    public void init_loop() {
        telemetryM.debug("The robot will run forward until it reaches " + VELOCITY + " inches per second.");
        telemetryM.debug("Then, it will cut power from the drivetrain and roll to a stop.");
        telemetryM.debug("Make sure you have enough room.");
        telemetryM.debug("After stopping, the forward zero power acceleration (natural deceleration) will be displayed.");
        telemetryM.debug("Press B on Gamepad 1 to stop.");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    /** This starts the OpMode by setting the drive motors to run forward at full power. */
    @Override
    public void start() {
        follower.startTeleopDrive(false);
        follower.update();
        follower.setTeleOpDrive(1,0,0,true);
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad 1 will stop the OpMode. When the robot hits the specified velocity, the robot will
     * record its deceleration / negative acceleration until it stops. Then, it will average all the
     * recorded deceleration / negative acceleration and print that value.
     */
    @Override
    public void loop() {
        if (gamepad1.bWasPressed()) {
            stopRobot();
            requestOpModeStop();
        }

        follower.update();
        draw();

        Vector heading = new Vector(1.0, follower.getPose().getHeading());
        if (!end) {
            if (!stopping) {
                if (follower.getVelocity().dot(heading) > VELOCITY) {
                    previousVelocity = follower.getVelocity().dot(heading);
                    previousTimeNano = System.nanoTime();
                    stopping = true;
                    follower.setTeleOpDrive(0,0,0,true);
                }
            } else {
                double currentVelocity = follower.getVelocity().dot(heading);
                accelerations.add((currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / Math.pow(10.0, 9)));
                previousVelocity = currentVelocity;
                previousTimeNano = System.nanoTime();
                if (currentVelocity < follower.getConstraints().getVelocityConstraint()) {
                    end = true;
                }
            }
        } else {
            double average = 0;
            for (double acceleration : accelerations) {
                average += acceleration;
            }
            average /= accelerations.size();

            telemetryM.debug("Forward Zero Power Acceleration (Deceleration): " + average);
            telemetryM.debug("\n");
            telemetryM.debug("Press A to set the Forward Zero Power Acceleration temporarily (while robot remains on).");
            telemetryM.update(telemetry);

            if (gamepad1.aWasPressed()) {
                follower.getConstants().setForwardZeroPowerAcceleration(average);
                String message = "Forward Zero Power Acceleration: " + average;
                changes.add(message);
            }
        }
    }
}

/**
 * This is the LateralZeroPowerAccelerationTuner autonomous follower OpMode. This runs the robot
 * to the left until a specified velocity is achieved. Then, the robot cuts power to the motors, setting
 * them to zero power. The deceleration, or negative acceleration, is then measured until the robot
 * stops. The accelerations across the entire time the robot is slowing down is then averaged and
 * that number is then printed. This is used to determine how the robot will decelerate in the
 * forward direction when power is cut, making the estimations used in the calculations for the
 * drive Vector more accurate and giving better braking at the end of Paths.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/13/2024
 */
class LateralZeroPowerAccelerationTuner extends OpMode {
    private final ArrayList<Double> accelerations = new ArrayList<>();
    public static double VELOCITY = 30;
    private double previousVelocity;
    private long previousTimeNano;
    private boolean stopping;
    private boolean end;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /** This initializes the drive motors as well as the Panels telemetry. */
    @Override
    public void init_loop() {
        telemetryM.debug("The robot will run to the left until it reaches " + VELOCITY + " inches per second.");
        telemetryM.debug("Then, it will cut power from the drivetrain and roll to a stop.");
        telemetryM.debug("Make sure you have enough room.");
        telemetryM.debug("After stopping, the lateral zero power acceleration (natural deceleration) will be displayed.");
        telemetryM.debug("Press B on game pad 1 to stop.");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    /** This starts the OpMode by setting the drive motors to run forward at full power. */
    @Override
    public void start() {
        follower.startTeleopDrive(false);
        follower.update();
        follower.setTeleOpDrive(0,1,0,true);
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad 1 will stop the OpMode. When the robot hits the specified velocity, the robot will
     * record its deceleration / negative acceleration until it stops. Then, it will average all the
     * recorded deceleration / negative acceleration and print that value.
     */
    @Override
    public void loop() {
        if (gamepad1.bWasPressed()) {
            stopRobot();
            requestOpModeStop();
        }

        follower.update();
        draw();

        Vector heading = new Vector(1.0, follower.getPose().getHeading() - Math.PI / 2);
        if (!end) {
            if (!stopping) {
                if (Math.abs(follower.getVelocity().dot(heading)) > VELOCITY) {
                    previousVelocity = Math.abs(follower.getVelocity().dot(heading));
                    previousTimeNano = System.nanoTime();
                    stopping = true;
                    follower.setTeleOpDrive(0,0,0,true);
                }
            } else {
                double currentVelocity = Math.abs(follower.getVelocity().dot(heading));
                accelerations.add((currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / Math.pow(10.0, 9)));
                previousVelocity = currentVelocity;
                previousTimeNano = System.nanoTime();
                if (currentVelocity < follower.getConstraints().getVelocityConstraint()) {
                    end = true;
                }
            }
        } else {
            double average = 0;
            for (double acceleration : accelerations) {
                average += acceleration;
            }
            average /= accelerations.size();

            telemetryM.debug("Lateral Zero Power Acceleration (Deceleration): " + average);
            telemetryM.debug("\n");
            telemetryM.debug("Press A to set the Lateral Zero Power Acceleration temporarily (while robot remains on).");
            telemetryM.update(telemetry);

            if (gamepad1.aWasPressed()) {
                follower.getConstants().setLateralZeroPowerAcceleration(average);
                String message = "Lateral Zero Power Acceleration: " + average;
                changes.add(message);
            }
        }
    }
}

/**
 * This is the Translational PIDF Tuner OpMode. It will keep the robot in place.
 * The user should push the robot laterally to test the PIDF and adjust the PIDF values accordingly.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
class TranslationalTuner extends OpMode {
    public static double DISTANCE = 40;
    private boolean forward = true;

    private Path forwards;
    private Path backwards;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /** This initializes the Follower and creates the forward and backward Paths. */
    @Override
    public void init_loop() {
        telemetryM.debug("This will activate the translational PIDF(s)");
        telemetryM.debug("The robot will try to stay in place while you push it laterally.");
        telemetryM.debug("You can adjust the PIDF values to tune the robot's translational PIDF(s).");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {
        follower.deactivateAllPIDFs();
        follower.activateTranslational();
        forwards = new Path(new BezierLine(new Pose(72,72), new Pose(DISTANCE + 72,72)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Pose(DISTANCE + 72,72), new Pose(72,72)));
        backwards.setConstantHeadingInterpolation(0);
        follower.followPath(forwards);
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
    @Override
    public void loop() {
        follower.update();
        draw();

        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryM.debug("Push the robot laterally to test the Translational PIDF(s).");
        telemetryM.update(telemetry);
    }
}

/**
 * This is the Heading PIDF Tuner OpMode. It will keep the robot in place.
 * The user should try to turn the robot to test the PIDF and adjust the PIDF values accordingly.
 * It will try to keep the robot at a constant heading while the user tries to turn it.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
class HeadingTuner extends OpMode {
    public static double DISTANCE = 40;
    private boolean forward = true;

    private Path forwards;
    private Path backwards;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the Panels telemetry.
     */
    @Override
    public void init_loop() {
        telemetryM.debug("This will activate the heading PIDF(s).");
        telemetryM.debug("The robot will try to stay at a constant heading while you try to turn it.");
        telemetryM.debug("You can adjust the PIDF values to tune the robot's heading PIDF(s).");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {
        follower.deactivateAllPIDFs();
        follower.activateHeading();
        forwards = new Path(new BezierLine(new Pose(72,72), new Pose(DISTANCE + 72,72)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Pose(DISTANCE + 72,72), new Pose(72,72)));
        backwards.setConstantHeadingInterpolation(0);
        follower.followPath(forwards);
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    @Override
    public void loop() {
        follower.update();
        draw();

        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryM.debug("Turn the robot manually to test the Heading PIDF(s).");
        telemetryM.update(telemetry);
    }
}

/**
 * This is the Drive PIDF Tuner OpMode. It will run the robot in a straight line going forward and back.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
class DriveTuner extends OpMode {
    public static double DISTANCE = 80;
    private boolean forward = true;

    private PathChain forwards;
    private PathChain backwards;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the Panels telemetry.
     */
    @Override
    public void init_loop() {
        telemetryM.debug("This will run the robot in a straight line going " + DISTANCE + "inches forward.");
        telemetryM.debug("The robot will go forward and backward continuously along the path.");
        telemetryM.debug("Make sure you have enough room.");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {
        follower.deactivateAllPIDFs();
        follower.activateDrive();
        
        forwards = follower.pathBuilder()
                .setGlobalDeceleration()
                .addPath(new BezierLine(new Pose(72,72), new Pose(DISTANCE + 72,72)))
                .setConstantHeadingInterpolation(0)
                .build();

        backwards = follower.pathBuilder()
                .setGlobalDeceleration()
                .addPath(new BezierLine(new Pose(DISTANCE + 72,72), new Pose(72,72)))
                .setConstantHeadingInterpolation(0)
                .build();

        follower.followPath(forwards);
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    @Override
    public void loop() {
        follower.update();
        draw();

        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryM.debug("Driving forward?: " + forward);
        telemetryM.update(telemetry);
    }
}

/**
 * This is the Line Test Tuner OpMode. It will drive the robot forward and back
 * The user should push the robot laterally and angular to test out the drive, heading, and translational PIDFs.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
class Line extends OpMode {
    public static double DISTANCE = 60;
    private boolean forward = true;

    private Path forwards;
    private Path backwards;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /** This initializes the Follower and creates the forward and backward Paths. */
    @Override
    public void init_loop() {
        telemetryM.debug("This will activate all the PIDF(s)");
        telemetryM.debug("The robot will go forward and backward continuously along the path while correcting.");
        telemetryM.debug("You can adjust the PIDF values to tune the robot's drive PIDF(s).");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {
        follower.activateAllPIDFs();
        forwards = new Path(new BezierLine(new Pose(72,72), new Pose(DISTANCE + 72,72)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Pose(DISTANCE + 72,72), new Pose(72,72)));
        backwards.setConstantHeadingInterpolation(0);
        follower.followPath(forwards);
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
    @Override
    public void loop() {
        follower.update();
        draw();

        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryM.debug("Driving Forward?: " + forward);
        telemetryM.update(telemetry);
    }
}

/**
 * This is the Centripetal Tuner OpMode. It runs the robot in a specified distance
 * forward and to the left. On reaching the end of the forward Path, the robot runs the backward
 * Path the same distance back to the start. Rinse and repeat! This is good for testing a variety
 * of Vectors, like the drive Vector, the translational Vector, the heading Vector, and the
 * centripetal Vector.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
class CentripetalTuner extends OpMode {
    public static double DISTANCE = 40;
    private boolean forward = true;

    private Path forwards;
    private Path backwards;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /**
     * This initializes the Follower and creates the forward and backward Paths.
     * Additionally, this initializes the Panels telemetry.
     */
    @Override
    public void init_loop() {
        telemetryM.debug("This will run the robot in a curve going " + DISTANCE + " inches to the left and the same number of inches forward.");
        telemetryM.debug("The robot will go continuously along the path.");
        telemetryM.debug("Make sure you have enough room.");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {
        follower.activateAllPIDFs();
        forwards = new Path(new BezierCurve(new Pose(72,72), new Pose(Math.abs(DISTANCE) + 72,72), new Pose(Math.abs(DISTANCE) + 72,DISTANCE + 72)));
        backwards = new Path(new BezierCurve(new Pose(Math.abs(DISTANCE) + 72,DISTANCE + 72), new Pose(Math.abs(DISTANCE) + 72,72), new Pose(72,72)));

        backwards.setTangentHeadingInterpolation();
        backwards.reverseHeadingInterpolation();

        follower.followPath(forwards);
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    @Override
    public void loop() {
        follower.update();
        draw();
        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryM.debug("Driving away from the origin along the curve?: " + forward);
        telemetryM.update(telemetry);
    }
}

/**
 * This is the Triangle autonomous OpMode.
 * It runs the robot in a triangle, with the starting point being the bottom-middle point.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Samarth Mahapatra - 1002 CircuitRunners Robotics Surge
 * @version 1.0, 12/30/2024
 */
class Triangle extends OpMode {

    private final Pose startPose = new Pose(72, 72, Math.toRadians(0));
    private final Pose interPose = new Pose(24 + 72, -24 + 72, Math.toRadians(90));
    private final Pose endPose = new Pose(24 + 72, 24 + 72, Math.toRadians(45));

    private PathChain triangle;

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    @Override
    public void loop() {
        follower.update();
        draw();

        if (follower.atParametricEnd()) {
            follower.followPath(triangle, true);
        }
    }

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    @Override
    public void init_loop() {
        telemetryM.debug("This will run in a roughly triangular shape, starting on the bottom-middle point.");
        telemetryM.debug("So, make sure you have enough space to the left, front, and right to run the OpMode.");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    /** Creates the PathChain for the "triangle".*/
    @Override
    public void start() {
        follower.setStartingPose(startPose);

        triangle = follower.pathBuilder()
                .addPath(new BezierLine(startPose, interPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), interPose.getHeading())
                .addPath(new BezierLine(interPose, endPose))
                .setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
                .addPath(new BezierLine(endPose, startPose))
                .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
                .build();

        follower.followPath(triangle);
    }
}

/**
 * This is the Circle autonomous OpMode. It runs the robot in a PathChain that's actually not quite
 * a circle, but some Bezier curves that have control points set essentially in a square. However,
 * it turns enough to tune your centripetal force correction and some of your heading. Some lag in
 * heading is to be expected.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
class Circle extends OpMode {
    public static double RADIUS = 30;
    private PathChain circle;

    public void start() {
        circle = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(72, 72), new Pose(RADIUS + 72, 72), new Pose(RADIUS + 72, RADIUS + 72)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(72, RADIUS + 72))
                .addPath(new BezierCurve(new Pose(RADIUS + 72, RADIUS + 72), new Pose(RADIUS + 72, (2 * RADIUS) + 72), new Pose(72, (2 * RADIUS) + 72)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(72, RADIUS + 72))
                .addPath(new BezierCurve(new Pose(72, (2 * RADIUS) + 72), new Pose(-RADIUS + 72, (2 * RADIUS) + 72), new Pose(-RADIUS + 72, RADIUS + 72)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(72, RADIUS + 72))
                .addPath(new BezierCurve(new Pose(-RADIUS + 72, RADIUS + 72), new Pose(-RADIUS + 72, 72), new Pose(72, 72)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(72, RADIUS + 72))
                .build();
        follower.followPath(circle);
    }

    @Override
    public void init_loop() {
        telemetryM.debug("This will run in a roughly circular shape of radius " + RADIUS + ", starting on the right-most edge. ");
        telemetryM.debug("So, make sure you have enough space to the left, front, and back to run the OpMode.");
        telemetryM.debug("It will also continuously face the center of the circle to test your heading and centripetal correction.");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
        draw();

        if (follower.atParametricEnd()) {
            follower.followPath(circle);
        }
    }
}

/**
 * This is the Drawing class. It handles the drawing of stuff on Panels Dashboard, like the robot.
 *
 * @author Lazar - 19234
 * @version 1.1, 5/19/2025
 */
class Drawing {
    public static final double ROBOT_RADIUS = 9; // woah
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.75
    );

    /**
     * This prepares Panels Field for using Pedro Offsets
     */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);

        sendPacket();
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {

            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    public static void sendPacket() {
        panelsField.update();
    }
}

/**
 * PodPDFAutoTuner - Automatic swerve pod PDF (no I) tuner.
 *
 * This OpMode automatically tunes the P, D, and F coefficients for each swerve pod's
 * angle control servo. It runs through multiple phases sequentially:
 *
 * Phase 0: Encoder Voltage Calibration - Finds actual min/max voltages for each encoder
 * Phase 1: Feedforward (F) Tuning - Measures static friction threshold
 * Phase 2: P Tuning - Finds optimal proportional gain via step response
 * Phase 3: D Tuning - Finds optimal derivative gain to minimize overshoot
 * Phase 4: Validation - Verifies tuned values with test pattern
 * Phase 5: Results - Displays final values in copy-paste format
 *
 * SAFETY: Maximum servo power is limited. Press B at any time to abort.
 *
 * @author MOEbo Sapiens
 * @version 1.0
 */
class PodPDFAutoTuner extends OpMode {

    // ==================== CONFIGURATION ====================
    // Pod hardware names (must match PedroConstants)
    private static final String[] POD_NAMES = {"LF", "RF", "LB", "RB"};
    private static final String[] MOTOR_NAMES = {"sm2", "sm1", "sm3", "sm0"};
    private static final String[] SERVO_NAMES = {"ss2", "ss1", "ss3", "ss0"};
    private static final String[] ENCODER_NAMES = {"se2", "se1", "se3", "se0"};

    // Safety limits
    private static final double MAX_SERVO_POWER = 0.65;  // Never exceed this
    private static final double PHASE_TIMEOUT = 45.0;    // Max seconds per pod per phase

    // Encoder calibration settings
    private static final double ENCODER_CAL_POWER = 0.20;  // Slow spin for calibration
    private static final double ENCODER_CAL_DURATION = 5.0; // Seconds per pod (multiple rotations)

    // Feedforward tuning settings
    private static final double FF_RAMP_INCREMENT = 0.004;  // Power increment per step
    private static final double FF_RAMP_INTERVAL = 0.04;    // Seconds between increments
    private static final double FF_MOVEMENT_THRESHOLD = 0.4; // Degrees change to detect movement
    private static final double FF_SEARCH_RANGE = 0.25;     // Search ±25% around initial estimate
    private static final int FF_SEARCH_STEPS = 7;           // Number of F values to test

    // P tuning settings
    private static final double P_START = 0.001;
    private static final double P_END = 0.025;
    private static final double P_INCREMENT = 0.001;
    private static final double STEP_SIZE = 90.0;           // Step response angle
    private static final double STEP_TIMEOUT = 2.5;         // Max seconds per step test
    private static final double SETTLE_THRESHOLD = 2.0;     // Degrees for "settled"
    private static final double SETTLE_TIME_REQUIRED = 0.3; // Seconds within threshold

    // D tuning settings
    private static final double D_MAX_RATIO = 0.4;          // Max D as ratio of optimal P
    private static final int D_SEARCH_STEPS = 12;           // Number of D values to test

    // Validation settings
    private static final double[] VALIDATION_ANGLES = {45.0, 90.0, 135.0, -90.0, -45.0};

    // Reasonable value ranges (for sanity checks)
    private static final double MIN_REASONABLE_F = 0.01;
    private static final double MAX_REASONABLE_F = 0.20;
    private static final double MIN_REASONABLE_P = 0.001;
    private static final double MAX_REASONABLE_P = 0.030;
    private static final double MIN_REASONABLE_D = 0.0;
    private static final double MAX_REASONABLE_D = 0.010;

    // ==================== HARDWARE ====================
    private AnalogInput[] encoders = new AnalogInput[4];
    private CRServo[] servos = new CRServo[4];
    private DcMotorEx[] motors = new DcMotorEx[4];

    // ==================== RESULTS ====================
    private double[] voltageMin = {3.3, 3.3, 3.3, 3.3};
    private double[] voltageMax = {0.0, 0.0, 0.0, 0.0};
    private double[] optimalF = new double[4];
    private double[] optimalP = new double[4];
    private double[] optimalD = new double[4];

    // Performance metrics from validation
    private double[] validationRiseTime = new double[4];
    private double[] validationOvershoot = new double[4];
    private double[] validationSettleTime = new double[4];

    // ==================== STATE MACHINE ====================
    private enum Phase {
        INIT,
        ENCODER_CAL,
        FEEDFORWARD,
        P_TUNING,
        D_TUNING,
        VALIDATION,
        COMPLETE
    }

    private Phase phase = Phase.INIT;
    private int currentPod = 0;
    private boolean aborted = false;

    // ==================== TIMING ====================
    private ElapsedTime phaseTimer = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private ElapsedTime subTimer = new ElapsedTime();
    private double lastLoopTime = 0;
    private double deltaTime = 0;

    // ==================== ENCODER CAL STATE ====================
    // (uses phaseTimer for duration)

    // ==================== FEEDFORWARD STATE ====================
    private enum FFSubphase { RAMP_CW, RAMP_CCW, SEARCH, DONE }
    private FFSubphase ffSubphase = FFSubphase.RAMP_CW;
    private double ffRampPower = 0;
    private double ffThresholdCW = 0;
    private double ffThresholdCCW = 0;
    private double ffLastAngle = 0;
    private double ffSearchValues[] = new double[FF_SEARCH_STEPS];
    private double ffSearchScores[] = new double[FF_SEARCH_STEPS];
    private int ffSearchIndex = 0;
    private double ffBestF = 0;
    private double ffBestScore = Double.MAX_VALUE;

    // ==================== P TUNING STATE ====================
    private enum PTuneSubphase { SETTLE, STEP, MEASURE, NEXT }
    private PTuneSubphase pSubphase = PTuneSubphase.SETTLE;
    private double testP = P_START;
    private double pBestP = 0;
    private double pBestScore = Double.MAX_VALUE;
    private ArrayList<Double> pTestedValues = new ArrayList<>();
    private ArrayList<Double> pTestedScores = new ArrayList<>();

    // ==================== D TUNING STATE ====================
    private enum DTuneSubphase { SETTLE, STEP, MEASURE, NEXT }
    private DTuneSubphase dSubphase = DTuneSubphase.SETTLE;
    private double testD = 0;
    private double dBestD = 0;
    private double dBestScore = Double.MAX_VALUE;
    private ArrayList<Double> dTestedValues = new ArrayList<>();
    private ArrayList<Double> dTestedScores = new ArrayList<>();
    private int dStepCount = 0;

    // ==================== VALIDATION STATE ====================
    private enum ValSubphase { SETTLE, STEP, MEASURE, NEXT }
    private ValSubphase valSubphase = ValSubphase.SETTLE;
    private int valAngleIndex = 0;
    private double valTotalRiseTime = 0;
    private double valMaxOvershoot = 0;
    private double valTotalSettleTime = 0;
    private int valSuccessCount = 0;

    // ==================== STEP RESPONSE MEASUREMENT ====================
    private double stepStartAngle = 0;
    private double stepTargetAngle = 0;
    private double stepRiseTime = -1;
    private double stepOvershoot = 0;
    private double stepSettleTime = -1;
    private double stepMaxAngleReached = 0;
    private boolean stepReached90Percent = false;
    private double stepSettleStartTime = -1;
    private double stepLastError = 0;

    // ==================== PID CONTROL STATE ====================
    private double pidLastError = 0;
    private double pidLastTime = 0;

    // ==================== STATUS MESSAGES ====================
    private String statusMessage = "";
    private String warningMessage = "";

    @Override
    public void init() {
        // Initialize hardware
        boolean hardwareOk = true;
        for (int i = 0; i < 4; i++) {
            try {
                motors[i] = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[i]);
                servos[i] = hardwareMap.get(CRServo.class, SERVO_NAMES[i]);
                encoders[i] = hardwareMap.get(AnalogInput.class, ENCODER_NAMES[i]);

                // Set motors to float so they don't resist pod rotation
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motors[i].setPower(0);

                // Stop servos initially
                servos[i].setPower(0);
            } catch (Exception e) {
                hardwareOk = false;
                warningMessage = "Hardware init failed for pod " + POD_NAMES[i] + ": " + e.getMessage();
            }
        }

        if (!hardwareOk) {
            statusMessage = "HARDWARE ERROR - Check connections";
        } else {
            statusMessage = "Ready. Press START to begin tuning.";
        }

        phase = Phase.INIT;
        loopTimer.reset();
    }

    @Override
    public void init_loop() {
        telemetryM.debug("=== POD PDF AUTO-TUNER ===");
        telemetryM.debug("");
        telemetryM.debug("This will automatically tune P, D, and F for all 4 swerve pods.");
        telemetryM.debug("The robot will spin each pod through various tests.");
        telemetryM.debug("");
        telemetryM.debug("MAKE SURE THE ROBOT IS ON BLOCKS OR WHEELS ARE OFF THE GROUND!");
        telemetryM.debug("");
        telemetryM.debug("Press B at any time to abort.");
        telemetryM.debug("");
        if (!warningMessage.isEmpty()) {
            telemetryM.debug("WARNING: " + warningMessage);
        }
        telemetryM.debug("Status: " + statusMessage);
        telemetryM.update(telemetry);

        // Also update regular telemetry
        telemetry.addLine("=== POD PDF AUTO-TUNER ===");
        telemetry.addLine("");
        telemetry.addLine("This will automatically tune P, D, F for all pods.");
        telemetry.addLine("ENSURE ROBOT IS ON BLOCKS!");
        telemetry.addLine("");
        telemetry.addLine("Press B to abort at any time.");
        if (!warningMessage.isEmpty()) {
            telemetry.addLine("WARNING: " + warningMessage);
        }
        telemetry.addLine("Status: " + statusMessage);
        telemetry.update();
    }

    @Override
    public void start() {
        phase = Phase.ENCODER_CAL;
        currentPod = 0;
        phaseTimer.reset();
        loopTimer.reset();
        statusMessage = "Starting encoder calibration...";
    }

    @Override
    public void loop() {
        // Calculate delta time
        double currentTime = loopTimer.seconds();
        deltaTime = currentTime - lastLoopTime;
        lastLoopTime = currentTime;

        // Check for abort
        if (gamepad1.b) {
            abortTuning("User pressed B to abort");
            return;
        }

        // Phase timeout check
        if (phase != Phase.INIT && phase != Phase.COMPLETE && phaseTimer.seconds() > PHASE_TIMEOUT) {
            warningMessage = "Phase timeout for pod " + POD_NAMES[currentPod] + " - moving to next";
            advanceToNextPod();
        }

        // Run current phase
        switch (phase) {
            case INIT:
                // Shouldn't reach here after start()
                break;
            case ENCODER_CAL:
                runEncoderCalibration();
                break;
            case FEEDFORWARD:
                runFeedforwardTuning();
                break;
            case P_TUNING:
                runPTuning();
                break;
            case D_TUNING:
                runDTuning();
                break;
            case VALIDATION:
                runValidation();
                break;
            case COMPLETE:
                displayResults();
                break;
        }

        // Update telemetry
        updateTelemetry();
    }

    // ==================== PHASE 0: ENCODER CALIBRATION ====================

    private void runEncoderCalibration() {
        double elapsed = phaseTimer.seconds();

        if (elapsed < ENCODER_CAL_DURATION) {
            // Spin the servo at constant power
            servos[currentPod].setPower(ENCODER_CAL_POWER);

            // Track min/max voltage
            double voltage = encoders[currentPod].getVoltage();
            voltageMin[currentPod] = Math.min(voltageMin[currentPod], voltage);
            voltageMax[currentPod] = Math.max(voltageMax[currentPod], voltage);

            statusMessage = String.format("Encoder Cal %s: %.1f%% | V=[%.3f - %.3f]",
                POD_NAMES[currentPod],
                (elapsed / ENCODER_CAL_DURATION) * 100,
                voltageMin[currentPod],
                voltageMax[currentPod]);
        } else {
            // Stop servo
            servos[currentPod].setPower(0);

            // Validate results
            double range = voltageMax[currentPod] - voltageMin[currentPod];
            if (range < 2.5) {
                warningMessage = String.format("Pod %s encoder range too small (%.2fV). Check wiring!",
                    POD_NAMES[currentPod], range);
            }

            // Move to next pod or next phase
            currentPod++;
            if (currentPod >= 4) {
                phase = Phase.FEEDFORWARD;
                currentPod = 0;
                resetFeedforwardState();
            }
            phaseTimer.reset();
        }
    }

    // ==================== PHASE 1: FEEDFORWARD TUNING ====================

    private void resetFeedforwardState() {
        ffSubphase = FFSubphase.RAMP_CW;
        ffRampPower = 0;
        ffThresholdCW = 0;
        ffThresholdCCW = 0;
        ffLastAngle = getCurrentAngle(currentPod);
        ffSearchIndex = 0;
        ffBestF = 0;
        ffBestScore = Double.MAX_VALUE;
        subTimer.reset();
    }

    private void runFeedforwardTuning() {
        switch (ffSubphase) {
            case RAMP_CW:
                runFFRampCW();
                break;
            case RAMP_CCW:
                runFFRampCCW();
                break;
            case SEARCH:
                runFFSearch();
                break;
            case DONE:
                // Store result and move on
                optimalF[currentPod] = ffBestF;
                servos[currentPod].setPower(0);

                // Sanity check
                if (ffBestF < MIN_REASONABLE_F || ffBestF > MAX_REASONABLE_F) {
                    warningMessage = String.format("Pod %s F=%.4f outside normal range [%.2f-%.2f]",
                        POD_NAMES[currentPod], ffBestF, MIN_REASONABLE_F, MAX_REASONABLE_F);
                }

                currentPod++;
                if (currentPod >= 4) {
                    phase = Phase.P_TUNING;
                    currentPod = 0;
                    resetPTuningState();
                } else {
                    resetFeedforwardState();
                }
                phaseTimer.reset();
                break;
        }
    }

    private void runFFRampCW() {
        if (subTimer.seconds() >= FF_RAMP_INTERVAL) {
            subTimer.reset();

            double currentAngle = getCurrentAngle(currentPod);
            double angleDelta = Math.abs(normalizeAngle(currentAngle - ffLastAngle));

            // Check if movement detected
            if (angleDelta > FF_MOVEMENT_THRESHOLD && ffRampPower > 0.01) {
                ffThresholdCW = ffRampPower;
                ffRampPower = 0;
                servos[currentPod].setPower(0);
                ffSubphase = FFSubphase.RAMP_CCW;
                ffLastAngle = getCurrentAngle(currentPod);
                subTimer.reset();

                // Small delay before reversing
                try { Thread.sleep(200); } catch (Exception e) {}
            } else {
                // Increment power
                ffRampPower = Math.min(ffRampPower + FF_RAMP_INCREMENT, MAX_SERVO_POWER);
                servos[currentPod].setPower(ffRampPower);
            }

            ffLastAngle = currentAngle;
        }

        statusMessage = String.format("FF %s: Ramp CW power=%.3f", POD_NAMES[currentPod], ffRampPower);
    }

    private void runFFRampCCW() {
        if (subTimer.seconds() >= FF_RAMP_INTERVAL) {
            subTimer.reset();

            double currentAngle = getCurrentAngle(currentPod);
            double angleDelta = Math.abs(normalizeAngle(currentAngle - ffLastAngle));

            // Check if movement detected
            if (angleDelta > FF_MOVEMENT_THRESHOLD && Math.abs(ffRampPower) > 0.01) {
                ffThresholdCCW = Math.abs(ffRampPower);
                servos[currentPod].setPower(0);

                // Calculate initial F estimate (average of thresholds)
                double initialF = (ffThresholdCW + ffThresholdCCW) / 2.0;

                // Set up search values around the estimate
                double searchMin = initialF * (1 - FF_SEARCH_RANGE);
                double searchMax = initialF * (1 + FF_SEARCH_RANGE);
                for (int i = 0; i < FF_SEARCH_STEPS; i++) {
                    ffSearchValues[i] = searchMin + (searchMax - searchMin) * i / (FF_SEARCH_STEPS - 1);
                    ffSearchScores[i] = Double.MAX_VALUE;
                }

                ffSearchIndex = 0;
                ffSubphase = FFSubphase.SEARCH;
                resetStepMeasurement(currentPod, STEP_SIZE / 2); // Smaller step for F search
                subTimer.reset();
            } else {
                // Increment power (negative direction)
                ffRampPower = Math.max(ffRampPower - FF_RAMP_INCREMENT, -MAX_SERVO_POWER);
                servos[currentPod].setPower(ffRampPower);
            }

            ffLastAngle = currentAngle;
        }

        statusMessage = String.format("FF %s: Ramp CCW power=%.3f (CW thresh=%.3f)",
            POD_NAMES[currentPod], ffRampPower, ffThresholdCW);
    }

    private void runFFSearch() {
        // Run a step response with current F value
        double currentF = ffSearchValues[ffSearchIndex];
        boolean stepDone = runStepMeasurement(currentPod, 0.005, 0, currentF); // Use minimal P for F search

        if (stepDone) {
            // Score based on steady-state error and settling time
            double score = Math.abs(stepLastError) + stepSettleTime * 0.1;
            if (stepSettleTime < 0) score = Double.MAX_VALUE; // Didn't settle

            ffSearchScores[ffSearchIndex] = score;

            if (score < ffBestScore) {
                ffBestScore = score;
                ffBestF = currentF;
            }

            ffSearchIndex++;
            if (ffSearchIndex >= FF_SEARCH_STEPS) {
                ffSubphase = FFSubphase.DONE;
            } else {
                resetStepMeasurement(currentPod, STEP_SIZE / 2);
            }
        }

        statusMessage = String.format("FF %s: Search %d/%d F=%.4f (best=%.4f)",
            POD_NAMES[currentPod], ffSearchIndex + 1, FF_SEARCH_STEPS,
            ffSearchValues[Math.min(ffSearchIndex, FF_SEARCH_STEPS-1)], ffBestF);
    }

    // ==================== PHASE 2: P TUNING ====================

    private void resetPTuningState() {
        pSubphase = PTuneSubphase.SETTLE;
        testP = P_START;
        pBestP = P_START;
        pBestScore = Double.MAX_VALUE;
        pTestedValues.clear();
        pTestedScores.clear();
        subTimer.reset();
    }

    private void runPTuning() {
        switch (pSubphase) {
            case SETTLE:
                // Let the pod settle before starting
                servos[currentPod].setPower(0);
                if (subTimer.seconds() > 0.3) {
                    resetStepMeasurement(currentPod, STEP_SIZE);
                    pSubphase = PTuneSubphase.STEP;
                }
                break;

            case STEP:
                boolean done = runStepMeasurement(currentPod, testP, 0, optimalF[currentPod]);
                if (done) {
                    pSubphase = PTuneSubphase.MEASURE;
                }
                break;

            case MEASURE:
                // Calculate score for this P value
                // Lower is better: fast rise, low overshoot, fast settle
                double score;
                if (stepRiseTime < 0 || stepSettleTime < 0) {
                    // Didn't complete properly
                    score = Double.MAX_VALUE;
                } else {
                    // Score formula: prioritize settling, penalize overshoot heavily
                    double overshootPenalty = 1 + (stepOvershoot / 10.0) * (stepOvershoot / 10.0);
                    score = stepRiseTime * 0.3 + stepSettleTime * 0.7;
                    score *= overshootPenalty;

                    // Heavy penalty for excessive overshoot (oscillation indicator)
                    if (stepOvershoot > 20) {
                        score = Double.MAX_VALUE;
                    }
                }

                pTestedValues.add(testP);
                pTestedScores.add(score);

                if (score < pBestScore) {
                    pBestScore = score;
                    pBestP = testP;
                }

                pSubphase = PTuneSubphase.NEXT;
                break;

            case NEXT:
                servos[currentPod].setPower(0);

                // Check if we should continue
                testP += P_INCREMENT;

                // Stop if: exceeded range, or score is getting much worse (oscillating)
                boolean shouldStop = testP > P_END;
                if (pTestedScores.size() >= 3) {
                    double lastScore = pTestedScores.get(pTestedScores.size() - 1);
                    double prevScore = pTestedScores.get(pTestedScores.size() - 2);
                    // If last two scores are much worse than best, and trending worse, stop
                    if (lastScore > pBestScore * 3 && lastScore > prevScore) {
                        shouldStop = true;
                    }
                }

                if (shouldStop) {
                    optimalP[currentPod] = pBestP;

                    // Sanity check
                    if (pBestP < MIN_REASONABLE_P || pBestP > MAX_REASONABLE_P) {
                        warningMessage = String.format("Pod %s P=%.4f outside normal range",
                            POD_NAMES[currentPod], pBestP);
                    }

                    currentPod++;
                    if (currentPod >= 4) {
                        phase = Phase.D_TUNING;
                        currentPod = 0;
                        resetDTuningState();
                    } else {
                        resetPTuningState();
                    }
                    phaseTimer.reset();
                } else {
                    pSubphase = PTuneSubphase.SETTLE;
                    subTimer.reset();
                }
                break;
        }

        statusMessage = String.format("P Tune %s: P=%.4f (best=%.4f, score=%.2f)",
            POD_NAMES[currentPod], testP, pBestP, pBestScore);
    }

    // ==================== PHASE 3: D TUNING ====================

    private void resetDTuningState() {
        dSubphase = DTuneSubphase.SETTLE;
        testD = 0;
        dBestD = 0;
        dBestScore = Double.MAX_VALUE;
        dTestedValues.clear();
        dTestedScores.clear();
        dStepCount = 0;
        subTimer.reset();
    }

    private void runDTuning() {
        double dMax = optimalP[currentPod] * D_MAX_RATIO;
        double dIncrement = dMax / D_SEARCH_STEPS;

        switch (dSubphase) {
            case SETTLE:
                servos[currentPod].setPower(0);
                if (subTimer.seconds() > 0.3) {
                    resetStepMeasurement(currentPod, STEP_SIZE);
                    dSubphase = DTuneSubphase.STEP;
                }
                break;

            case STEP:
                boolean done = runStepMeasurement(currentPod, optimalP[currentPod], testD, optimalF[currentPod]);
                if (done) {
                    dSubphase = DTuneSubphase.MEASURE;
                }
                break;

            case MEASURE:
                // Calculate score - prioritize low overshoot more than P tuning
                double score;
                if (stepRiseTime < 0 || stepSettleTime < 0) {
                    score = Double.MAX_VALUE;
                } else {
                    // Heavily penalize overshoot, want minimal
                    double overshootPenalty = 1 + stepOvershoot * stepOvershoot / 25.0;
                    score = stepSettleTime * overshootPenalty;

                    // Also penalize very slow rise (too much D)
                    if (stepRiseTime > 1.0) {
                        score *= (1 + stepRiseTime - 1.0);
                    }
                }

                dTestedValues.add(testD);
                dTestedScores.add(score);

                if (score < dBestScore) {
                    dBestScore = score;
                    dBestD = testD;
                }

                dSubphase = DTuneSubphase.NEXT;
                break;

            case NEXT:
                servos[currentPod].setPower(0);
                dStepCount++;
                testD += dIncrement;

                // Stop if exceeded range or response is getting sluggish
                boolean shouldStop = dStepCount >= D_SEARCH_STEPS;
                if (dTestedScores.size() >= 2) {
                    double lastScore = dTestedScores.get(dTestedScores.size() - 1);
                    // If last score is much worse, we've gone too far
                    if (lastScore > dBestScore * 2.5 && lastScore != Double.MAX_VALUE) {
                        shouldStop = true;
                    }
                }

                if (shouldStop) {
                    optimalD[currentPod] = dBestD;

                    currentPod++;
                    if (currentPod >= 4) {
                        phase = Phase.VALIDATION;
                        currentPod = 0;
                        resetValidationState();
                    } else {
                        resetDTuningState();
                    }
                    phaseTimer.reset();
                } else {
                    dSubphase = DTuneSubphase.SETTLE;
                    subTimer.reset();
                }
                break;
        }

        statusMessage = String.format("D Tune %s: D=%.5f (best=%.5f)",
            POD_NAMES[currentPod], testD, dBestD);
    }

    // ==================== PHASE 4: VALIDATION ====================

    private void resetValidationState() {
        valSubphase = ValSubphase.SETTLE;
        valAngleIndex = 0;
        valTotalRiseTime = 0;
        valMaxOvershoot = 0;
        valTotalSettleTime = 0;
        valSuccessCount = 0;
        subTimer.reset();
    }

    private void runValidation() {
        switch (valSubphase) {
            case SETTLE:
                servos[currentPod].setPower(0);
                if (subTimer.seconds() > 0.3) {
                    resetStepMeasurement(currentPod, VALIDATION_ANGLES[valAngleIndex]);
                    valSubphase = ValSubphase.STEP;
                }
                break;

            case STEP:
                boolean done = runStepMeasurement(currentPod,
                    optimalP[currentPod], optimalD[currentPod], optimalF[currentPod]);
                if (done) {
                    valSubphase = ValSubphase.MEASURE;
                }
                break;

            case MEASURE:
                if (stepRiseTime > 0 && stepSettleTime > 0) {
                    valTotalRiseTime += stepRiseTime;
                    valMaxOvershoot = Math.max(valMaxOvershoot, stepOvershoot);
                    valTotalSettleTime += stepSettleTime;
                    valSuccessCount++;
                }
                valSubphase = ValSubphase.NEXT;
                break;

            case NEXT:
                servos[currentPod].setPower(0);
                valAngleIndex++;

                if (valAngleIndex >= VALIDATION_ANGLES.length) {
                    // Calculate averages
                    if (valSuccessCount > 0) {
                        validationRiseTime[currentPod] = valTotalRiseTime / valSuccessCount;
                        validationOvershoot[currentPod] = valMaxOvershoot;
                        validationSettleTime[currentPod] = valTotalSettleTime / valSuccessCount;
                    }

                    currentPod++;
                    if (currentPod >= 4) {
                        phase = Phase.COMPLETE;
                        stopAllServos();
                    } else {
                        resetValidationState();
                    }
                    phaseTimer.reset();
                } else {
                    valSubphase = ValSubphase.SETTLE;
                    subTimer.reset();
                }
                break;
        }

        statusMessage = String.format("Validate %s: angle %d/%d",
            POD_NAMES[currentPod], valAngleIndex + 1, VALIDATION_ANGLES.length);
    }

    // ==================== STEP RESPONSE MEASUREMENT ====================

    private void resetStepMeasurement(int pod, double stepAngle) {
        stepStartAngle = getCurrentAngle(pod);
        stepTargetAngle = stepStartAngle + stepAngle;
        stepRiseTime = -1;
        stepOvershoot = 0;
        stepSettleTime = -1;
        stepMaxAngleReached = stepStartAngle;
        stepReached90Percent = false;
        stepSettleStartTime = -1;
        pidLastError = 0;
        pidLastTime = loopTimer.seconds();
        subTimer.reset();
    }

    /**
     * Runs one iteration of step response measurement.
     * Returns true when the measurement is complete.
     */
    private boolean runStepMeasurement(int pod, double kP, double kD, double kF) {
        double currentAngle = getCurrentAngle(pod);
        double currentTime = loopTimer.seconds();
        double dt = currentTime - pidLastTime;
        pidLastTime = currentTime;

        // Calculate error (with proper angle wrapping)
        double error = normalizeAngle(stepTargetAngle - currentAngle);

        // Calculate derivative
        double derivative = 0;
        if (dt > 0.001) {
            derivative = (error - pidLastError) / dt;
        }
        pidLastError = error;

        // PD + F control
        double power = kP * error + kD * derivative;

        // Add feedforward if error is significant
        if (Math.abs(error) > 0.5) {
            power += kF * Math.signum(power);
        }

        // Clamp power
        power = Math.max(-MAX_SERVO_POWER, Math.min(MAX_SERVO_POWER, power));
        servos[pod].setPower(power);

        // Track max angle (for overshoot calculation)
        double distanceFromStart = Math.abs(normalizeAngle(currentAngle - stepStartAngle));
        double targetDistance = Math.abs(normalizeAngle(stepTargetAngle - stepStartAngle));

        if (distanceFromStart > stepMaxAngleReached) {
            stepMaxAngleReached = distanceFromStart;
        }

        // Check for 90% of target reached (for rise time)
        if (!stepReached90Percent && distanceFromStart >= targetDistance * 0.9) {
            stepReached90Percent = true;
            stepRiseTime = subTimer.seconds();
        }

        // Check if within settle threshold
        if (Math.abs(error) < SETTLE_THRESHOLD) {
            if (stepSettleStartTime < 0) {
                stepSettleStartTime = subTimer.seconds();
            } else if (subTimer.seconds() - stepSettleStartTime >= SETTLE_TIME_REQUIRED) {
                // Settled!
                stepSettleTime = subTimer.seconds();
                stepOvershoot = Math.max(0, stepMaxAngleReached - targetDistance);
                stepLastError = error;
                return true;
            }
        } else {
            stepSettleStartTime = -1; // Reset settle timer if we leave threshold
        }

        // Timeout
        if (subTimer.seconds() > STEP_TIMEOUT) {
            stepOvershoot = Math.max(0, stepMaxAngleReached - targetDistance);
            stepLastError = error;
            return true;
        }

        return false;
    }

    // ==================== UTILITY METHODS ====================

    private double getCurrentAngle(int pod) {
        double voltage = encoders[pod].getVoltage();
        double range = voltageMax[pod] - voltageMin[pod];

        // Handle uninitialized voltage range
        if (range < 0.1) {
            range = 3.3; // Default
            voltageMin[pod] = 0;
        }

        double normalized = (voltage - voltageMin[pod]) / range;
        normalized = Math.max(0, Math.min(1, normalized));
        return normalized * 360.0;
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private void stopAllServos() {
        for (int i = 0; i < 4; i++) {
            servos[i].setPower(0);
        }
    }

    private void abortTuning(String reason) {
        aborted = true;
        stopAllServos();
        phase = Phase.COMPLETE;
        warningMessage = "ABORTED: " + reason;
    }

    private void advanceToNextPod() {
        stopAllServos();
        currentPod++;

        if (currentPod >= 4) {
            // Move to next phase
            switch (phase) {
                case ENCODER_CAL:
                    phase = Phase.FEEDFORWARD;
                    currentPod = 0;
                    resetFeedforwardState();
                    break;
                case FEEDFORWARD:
                    phase = Phase.P_TUNING;
                    currentPod = 0;
                    resetPTuningState();
                    break;
                case P_TUNING:
                    phase = Phase.D_TUNING;
                    currentPod = 0;
                    resetDTuningState();
                    break;
                case D_TUNING:
                    phase = Phase.VALIDATION;
                    currentPod = 0;
                    resetValidationState();
                    break;
                case VALIDATION:
                    phase = Phase.COMPLETE;
                    break;
                default:
                    break;
            }
        } else {
            // Reset state for new pod in current phase
            switch (phase) {
                case ENCODER_CAL:
                    break; // No special reset needed
                case FEEDFORWARD:
                    resetFeedforwardState();
                    break;
                case P_TUNING:
                    resetPTuningState();
                    break;
                case D_TUNING:
                    resetDTuningState();
                    break;
                case VALIDATION:
                    resetValidationState();
                    break;
                default:
                    break;
            }
        }
        phaseTimer.reset();
    }

    // ==================== RESULTS DISPLAY ====================

    private void displayResults() {
        // Results are displayed in updateTelemetry when phase == COMPLETE
    }

    // ==================== TELEMETRY ====================

    private void updateTelemetry() {
        // Panels telemetry
        telemetryM.debug("=== POD PDF AUTO-TUNER ===");
        telemetryM.debug("");

        if (phase == Phase.COMPLETE) {
            if (aborted) {
                telemetryM.debug("*** TUNING ABORTED ***");
                telemetryM.debug(warningMessage);
                telemetryM.debug("");
            } else {
                telemetryM.debug("*** TUNING COMPLETE ***");
            }

            telemetryM.debug("");
            telemetryM.debug("=== COPY TO PedroConstants.java ===");
            telemetryM.debug("");

            for (int i = 0; i < 4; i++) {
                telemetryM.debug(String.format("// %s Pod", POD_NAMES[i]));
                telemetryM.debug(String.format("new PIDFCoefficients(%.4f, 0, %.5f, %.4f)",
                    optimalP[i], optimalD[i], optimalF[i]));
                telemetryM.debug(String.format("// Voltage range: %.3f - %.3f",
                    voltageMin[i], voltageMax[i]));
                telemetryM.debug("");
            }

            telemetryM.debug("=== PERFORMANCE SUMMARY ===");
            for (int i = 0; i < 4; i++) {
                telemetryM.debug(String.format("%s: Rise=%.0fms Overshoot=%.1f° Settle=%.0fms",
                    POD_NAMES[i],
                    validationRiseTime[i] * 1000,
                    validationOvershoot[i],
                    validationSettleTime[i] * 1000));
            }
        } else {
            // Show current phase and progress
            telemetryM.debug("Phase: " + phase.name());
            telemetryM.debug("Pod: " + POD_NAMES[Math.min(currentPod, 3)] + " (" + (currentPod + 1) + "/4)");
            telemetryM.debug("");
            telemetryM.debug(statusMessage);
            telemetryM.debug("");

            // Show current angle and power for active pod
            if (currentPod < 4) {
                telemetryM.debug(String.format("Angle: %.1f°", getCurrentAngle(currentPod)));
                telemetryM.debug(String.format("Servo Power: %.3f", servos[currentPod].getPower()));
            }

            telemetryM.debug("");
            telemetryM.debug("--- Discovered Values ---");
            for (int i = 0; i < 4; i++) {
                String pStr = (phase.ordinal() > Phase.P_TUNING.ordinal() ||
                              (phase == Phase.P_TUNING && i < currentPod))
                              ? String.format("%.4f", optimalP[i]) : "---";
                String dStr = (phase.ordinal() > Phase.D_TUNING.ordinal() ||
                              (phase == Phase.D_TUNING && i < currentPod))
                              ? String.format("%.5f", optimalD[i]) : "---";
                String fStr = (phase.ordinal() > Phase.FEEDFORWARD.ordinal() ||
                              (phase == Phase.FEEDFORWARD && i < currentPod))
                              ? String.format("%.4f", optimalF[i]) : "---";
                String vStr = (phase.ordinal() > Phase.ENCODER_CAL.ordinal() ||
                              (phase == Phase.ENCODER_CAL && i < currentPod))
                              ? String.format("[%.2f-%.2f]", voltageMin[i], voltageMax[i]) : "---";

                telemetryM.debug(String.format("%s: P=%s D=%s F=%s V=%s",
                    POD_NAMES[i], pStr, dStr, fStr, vStr));
            }

            if (!warningMessage.isEmpty()) {
                telemetryM.debug("");
                telemetryM.debug("WARN: " + warningMessage);
            }
        }

        telemetryM.debug("");
        telemetryM.debug("Press B to abort");
        telemetryM.update(telemetry);

        // Also update regular FTC telemetry (Driver Station)
        telemetry.addLine("=== POD PDF AUTO-TUNER ===");
        telemetry.addLine("");

        if (phase == Phase.COMPLETE) {
            if (aborted) {
                telemetry.addLine("*** ABORTED ***");
                telemetry.addLine(warningMessage);
            } else {
                telemetry.addLine("*** COMPLETE ***");
            }
            telemetry.addLine("");
            telemetry.addLine("== RESULTS (copy to PedroConstants) ==");
            for (int i = 0; i < 4; i++) {
                telemetry.addLine(String.format("%s: P=%.4f D=%.5f F=%.4f",
                    POD_NAMES[i], optimalP[i], optimalD[i], optimalF[i]));
                telemetry.addLine(String.format("   V=[%.3f-%.3f]", voltageMin[i], voltageMax[i]));
            }
            telemetry.addLine("");
            telemetry.addLine("== PERFORMANCE ==");
            for (int i = 0; i < 4; i++) {
                telemetry.addLine(String.format("%s: %.0fms rise, %.1f° overshoot",
                    POD_NAMES[i], validationRiseTime[i] * 1000, validationOvershoot[i]));
            }
        } else {
            telemetry.addLine("Phase: " + phase.name());
            telemetry.addLine("Pod: " + POD_NAMES[Math.min(currentPod, 3)] + " (" + (currentPod + 1) + "/4)");
            telemetry.addLine("");
            telemetry.addLine(statusMessage);

            if (currentPod < 4) {
                telemetry.addLine("");
                telemetry.addLine(String.format("Angle: %.1f°  Power: %.3f",
                    getCurrentAngle(currentPod), servos[currentPod].getPower()));
            }

            telemetry.addLine("");
            telemetry.addLine("-- Current Values --");
            for (int i = 0; i < 4; i++) {
                boolean hasPDF = phase.ordinal() > Phase.D_TUNING.ordinal() ||
                                (phase == Phase.D_TUNING && i < currentPod);
                if (hasPDF) {
                    telemetry.addLine(String.format("%s: P=%.4f D=%.5f F=%.4f",
                        POD_NAMES[i], optimalP[i], optimalD[i], optimalF[i]));
                }
            }

            if (!warningMessage.isEmpty()) {
                telemetry.addLine("");
                telemetry.addLine("WARN: " + warningMessage);
            }
        }

        telemetry.addLine("");
        telemetry.addLine("Press B to abort");
        telemetry.update();
    }

    @Override
    public void stop() {
        stopAllServos();
    }
}
