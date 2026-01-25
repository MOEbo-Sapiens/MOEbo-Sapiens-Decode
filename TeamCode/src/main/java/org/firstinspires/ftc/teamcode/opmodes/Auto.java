package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.Scheduler.cancel;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.States;

public abstract class Auto extends LinearOpMode {
    Robot robot;
    //default for all poses is blue side

    //length = 15.39
    //width = 15.12

    private Command updateShooter;

    protected Pose startPose = new Pose(20.8, 124.1, Math.toRadians(234)); //TODO: actually determine
    protected Pose shootingPose = new Pose(59, 85, Math.toRadians(180));
    protected Pose middlePickupPose = new Pose(15, 58, Math.toRadians(180));
    protected Pose middlePickupControlPoint = new Pose(59, 48);
    protected Pose gatePickupPose = new Pose(16, 60, Math.toRadians(150));
    protected Pose gatePickupControlPoint = new Pose(44, 67);
    protected Pose closePickupPose = new Pose(24, 85, Math.toRadians(180));
    protected Pose gateClearControlPoint = new Pose(40, 72);
    protected Pose gateClearPose = new Pose(18, 75, Math.toRadians(90));
    protected Pose farPickupPose = new Pose(15, 35, Math.toRadians(180));
    protected Pose farPickupControlPoint = new Pose(72, 20);
    protected Pose cornerPose = new Pose(15, 10, Math.toRadians(270));
    protected Pose cornerControlPoint = new Pose(12, 78);
    protected Pose cornerBackupControlPoint = new Pose(15, 52);
    protected Pose parkPose = new Pose(59, 109, Math.toRadians(180));
    protected Pose goalPose = Constants.BLUE_GOAL_POSE;

    PathChain shootPreloads;
    PathChain pickupMiddle;
    PathChain shootMiddle;
    PathChain pickupGate1;
    PathChain shootGate1;
    PathChain pickupGate2;
    PathChain shootGate2;
    PathChain pickupClose;
    PathChain shootClose;
    PathChain pickupFar;
    PathChain shootFar;
    PathChain clearGate;
    PathChain pickupCorner;
    PathChain backupCorner;
    PathChain shootCorner;

    abstract void setPoses();
    abstract void setColor();

    private void createAutoCommands() {
        updateShooter = robot.updateShootingSubsystems();

        schedule(sequential(
                parallel(
                        follow(robot.getFollower(), shootPreloads),
                        sequential(
                                robot.setIntakePower(0.5),
                                waitMs(200),
                                setShooting()
                        )
                ),
                waitMs(250),
                shootAndSetIntaking(),
//                waitMs(500),

                parallel(
                        follow(robot.getFollower(), pickupMiddle),
                        sequential(
                                waitMs(500),
                                robot.setIntakePower(1)
                        )
                ),
                waitMs(200),
                setShooting(),
                follow(robot.getFollower(), shootMiddle),
                shootAndSetIntaking(),
//                waitMs(500),

                parallel(
                        follow(robot.getFollower(), pickupClose),
                        sequential(
                                waitMs(500),
                                robot.setIntakePower(1)
                        )
                ),
                waitMs(200),

                follow(robot.getFollower(), clearGate),
                waitMs(200),

                setShooting(),
                follow(robot.getFollower(), shootClose),
                waitMs(500),
                shootAndSetIntaking(),
//                waitMs(500),

                parallel(
                        follow(robot.getFollower(), pickupFar),
                        sequential(
                                waitMs(500),
                                robot.setIntakePower(1)
                        )
                ),
                waitMs(200),
                parallel(
                        sequential(
                                waitMs(1000),
                                setShooting()
                        ),
                        follow(robot.getFollower(), shootFar)
                ),
                waitMs(250),
                shootAndSetIntaking(),
//                waitMs(500),


                parallel(
                        sequential(
                                follow(robot.getFollower(), pickupCorner),
                                waitMs(200),
                                follow(robot.getFollower(), backupCorner)
                        ),
                        sequential(
                                waitMs(500),
                                robot.setIntakePower(1)
                        )
                ),
                waitMs(200),
                parallel(
                        sequential(
                                waitMs(1250),
                                setShooting()
                        ),
                        follow(robot.getFollower(), shootCorner)
                ),
                shootAndSetIntaking(),
//                waitMs(500),
                robot.setIntakePower(0)
        ));
    }

    private Command setShooting() {
        return sequential(
                robot.setIntakePower(0.5),
                instant(() -> schedule(updateShooter)),
                instant(() -> robot.setState(States.SHOOTING))
        );
    }

    private Command shootAndSetIntaking() {
        return sequential(
                        waitUntil(robot::readyToShoot).raceWith(infinite(() -> {
                            telemetry.addData("Waiting to shoot...", "");
                        })).raceWith(waitMs(5000)),
                        robot.shootMotif(),
                        instant(() -> cancel(updateShooter)),
                        instant(() -> robot.setState(States.INTAKING))
                );
    }

    private void generatePaths() {
        shootPreloads = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(startPose, shootingPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootingPose.getHeading())
                .build();

        pickupMiddle = robot.getFollower().pathBuilder()
                .addPath( new BezierCurve(shootingPose,
                        middlePickupControlPoint,
                        middlePickupPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

        shootMiddle = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(middlePickupPose,
                        middlePickupControlPoint,
                        shootingPose
                )).setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

        pickupGate1 = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(shootingPose,
                        gatePickupControlPoint,
                        gatePickupPose
                )).setLinearHeadingInterpolation(shootingPose.getHeading(), gatePickupPose.getHeading())
                .build();

        shootGate1 = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(gatePickupPose,
                        gatePickupControlPoint,
                        shootingPose
                )).setConstantHeadingInterpolation(gatePickupPose.getHeading())
                .build();


        pickupGate2 = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(shootingPose,
                        gatePickupControlPoint,
                        gatePickupPose
                )).setConstantHeadingInterpolation(gatePickupPose.getHeading())
                .build();

        shootGate2 = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(gatePickupPose,
                        gatePickupControlPoint,
                        shootingPose
                )).setLinearHeadingInterpolation(gatePickupPose.getHeading(), shootingPose.getHeading())
                .build();


        pickupClose = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(shootingPose, closePickupPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

        clearGate = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(closePickupPose, gateClearControlPoint,gateClearPose))
                .setLinearHeadingInterpolation(closePickupPose.getHeading(), gateClearPose.getHeading())
                .build();

        shootClose = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(gateClearPose, shootingPose))
                .setLinearHeadingInterpolation(gateClearPose.getHeading(), shootingPose.getHeading())
                .build();

        pickupFar = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(shootingPose, farPickupControlPoint, farPickupPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

        shootFar = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(farPickupPose, farPickupControlPoint, shootingPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

        pickupCorner = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(shootingPose, cornerControlPoint, cornerPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), cornerPose.getHeading())
                .build();

        backupCorner = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(cornerPose, cornerBackupControlPoint, cornerPose))
                .setConstantHeadingInterpolation(cornerPose.getHeading())
                .build();

        shootCorner = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(cornerPose, cornerControlPoint, parkPose))
                .setLinearHeadingInterpolation(cornerPose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void initialize() {
        Constants.reset();
        setColor();
        setPoses();
        Constants.lastOpModeWasAuto = true;
        Scheduler.reset();

        robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry, goalPose);
        Constants.robot = robot;
        robot.setPose(startPose);
        robot.init();

        generatePaths();
    }

    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        createAutoCommands();
        while (opModeIsActive()) {
            Scheduler.execute();
        }
    }
}
