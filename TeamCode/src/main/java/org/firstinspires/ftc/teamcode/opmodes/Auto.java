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
import com.pedropathing.paths.PathConstraints;
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
    protected Pose shootingPose = new Pose(55, 83, Math.toRadians(180));
    protected Pose middlePickupPose = new Pose(16, 58, Math.toRadians(180));
    protected Pose middlePickupControlPoint = new Pose(59, 48);
    protected Pose closePickupPose = new Pose(24, 85, Math.toRadians(180));
    protected Pose gateClearControlPoint = new Pose(55, 65);
    protected Pose gateClearPose = new Pose(22.5, 65.78, Math.toRadians(123.4));
    protected Pose gatePickupControlPoint = new Pose(25, 57);
    protected Pose gatePickupPose = new Pose(17, 53, Math.toRadians(150));
    protected Pose farPickupPose = new Pose(16, 35, Math.toRadians(180));
    protected Pose farPickupControlPoint = new Pose(72, 20);
//    protected Pose cornerPose = new Pose(15, 10, Math.toRadians(270));
//    protected Pose cornerControlPoint = new Pose(12, 78);
//    protected Pose cornerBackupControlPoint = new Pose(15, 52);
    protected Pose parkPose = new Pose(59, 109, Math.toRadians(180));
    protected Pose goalPose = Constants.BLUE_GOAL_POSE;

    PathChain shootPreloads;
    PathChain pickupMiddle;
    PathChain shootMiddle;
    PathChain clearGate1;
    PathChain pickupGate;
    PathChain shootGate1;
    PathChain clearGate2;
    PathChain shootGate2;
    PathChain pickupClose;
    PathChain shootClose;
    PathChain pickupFar;
    PathChain shootFar;
//    PathChain clearGate;
//    PathChain pickupCorner;
//    PathChain backupCorner;
//    PathChain shootCorner;

    abstract void setPoses();
    abstract void setColor();

    private void createAutoCommands() {
//        robot.getFollower().setMaxPower(0.9);
        updateShooter = robot.updateShootingSubsystems();

        double shootTime = 750;

        schedule(sequential(
                //Shoot preload
                parallel(
                        follow(robot.getFollower(), shootPreloads),
                        sequential(
                                robot.setIntakePower(0.4),
                                waitMs(200),
                                setShooting()
                        )
                ),
//                waitMs(300),

                parallel(
                        shootAndSetIntaking(),
                        //pickup and shoot middle
                        sequential(
                                waitMs(shootTime),
                                parallel(
                                        follow(robot.getFollower(), pickupMiddle),
                                        sequential(
                                                waitMs(700),
                                                robot.setIntakePower(1)
                                        )
                                )
                        )
                ),
//                waitMs(600),
                parallel(
                        follow(robot.getFollower(), shootMiddle),
                        sequential(
                                waitMs(100),
                                setShooting()
                        )
                ),
//                waitMs(200),


                parallel(
                        shootAndSetIntaking(),
                        //first gate clear / pickup + shoot
                        sequential(
                                waitMs(shootTime),
                                follow(robot.getFollower(), clearGate1)
//                                parallel(
//                                        follow(robot.getFollower(), clearGate1),
//                                        sequential(
//                                                waitUntil(() -> robot.getFollower().getCurrentTValue() > 0.85),
//                                                instant(() -> robot.getFollower().breakFollowing())
//                                        )
//                                )
                        )
                ),
                robot.setIntakePower(1),
//                waitMs(500),

                follow(robot.getFollower(), pickupGate),
                waitMs(1000),

                parallel(
                        follow(robot.getFollower(), shootGate1),
                        sequential(
                                waitMs(900),
                                setShooting()
                        )
                ),
//                waitMs(200),
                shootAndSetIntaking(),
                waitMs(750),

                //second gate clear / pickup + shoot
                follow(robot.getFollower(), clearGate2),
//                parallel(
//                        follow(robot.getFollower(), clearGate2),
//                        sequential(
//                                waitUntil(() -> robot.getFollower().getCurrentTValue() > 0.85),
//                                instant(() -> robot.getFollower().breakFollowing())
//                        )
//                ),
                robot.setIntakePower(1),
//                waitMs(500),

                follow(robot.getFollower(), pickupGate),
                waitMs(1000),

                parallel(
                        follow(robot.getFollower(), shootGate2),
                        sequential(
                                waitMs(900),
                                setShooting()
                        )
                ),
//                waitMs(200),

                parallel(
                        shootAndSetIntaking(),
                        //shoot and pickup close
                        sequential(
                                waitMs(shootTime),
                                parallel(
                                        follow(robot.getFollower(), pickupClose),
                                        sequential(
                                                waitMs(750),
                                                robot.setIntakePower(1)
                                        )
                                )
                        )
                ),
                waitMs(300),


                parallel(
                        follow(robot.getFollower(), shootClose),
                        sequential(
                                waitMs(200),
                                setShooting()
                        )
                ),
//                waitMs(200),

                parallel(
                        shootAndSetIntaking(),
                        //shoot and pickup far
                        sequential(
                                waitMs(shootTime),
                                parallel(
                                        follow(robot.getFollower(), pickupFar),
                                        sequential(
                                                waitMs(700),
                                                robot.setIntakePower(1)
                                        )
                                )
                        )
                ),
//                waitMs(600),
                parallel(
                        sequential(
                                waitMs(1000),
                                setShooting()
                        ),
                        follow(robot.getFollower(), shootFar)
                ),
//                waitMs(400),
                shootAndSetIntaking(),


//                parallel(
//                        sequential(
//                                follow(robot.getFollower(), pickupCorner)
////                                waitMs(200),
////                                follow(robot.getFollower(), backupCorner)
//                        ),
//                        sequential(
//                                waitMs(500),
//                                robot.setIntakePower(1)
//                        )
//                ),
//                waitMs(200),
//                parallel(
//                        sequential(
//                                waitMs(1250),
//                                setShooting()
//                        ),
//                        follow(robot.getFollower(), shootCorner)
//                ),
//                shootAndSetIntaking(),
//                waitMs(500),
                robot.setIntakePower(0),
                robot.setTurretPos(0)
        ));
    }

    private Command setShooting() {
        return sequential(
                robot.setIntakePower(0.4),
                instant(() -> schedule(updateShooter)),
                instant(() -> robot.setState(States.SHOOTING))
        );
    }

    private Command shootAndSetIntaking() {
        return sequential(
//                        waitUntil(robot::readyToShoot).raceWith(infinite(() -> {
//                            telemetry.addData("Waiting to shoot...", "");
//                        })).raceWith(waitMs(500)),
                        robot.shootMotif(500),
                        instant(() -> cancel(updateShooter)),
                        instant(() -> robot.setState(States.INTAKING))
                );
    }

    private void generatePaths() {
        shootPreloads = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(startPose, shootingPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootingPose.getHeading())
                .setTValueConstraint(0.8)
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

        clearGate1 = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(shootingPose,
                        gateClearControlPoint,
                        gateClearPose
                )).setLinearHeadingInterpolation(shootingPose.getHeading(), gateClearPose.getHeading())
                .setTValueConstraint(0.8)
                .build();

//        pickupGate = robot.getFollower().pathBuilder()
//                .addPath(new BezierCurve(gateClearPose,
//                        gatePickupControlPoint,
//                        gatePickupPose
//                )).setConstantHeadingInterpolation(gateClearPose.getHeading())
//                .build();

        pickupGate = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(gateClearPose, gatePickupControlPoint, gatePickupPose))
                .setLinearHeadingInterpolation(gateClearPose.getHeading(), gatePickupPose.getHeading())
                .build();

        shootGate1 = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(gatePickupPose,
                        shootingPose
                )).setLinearHeadingInterpolation(gatePickupPose.getHeading(), gateClearPose.getHeading())
                .build();


        clearGate2 = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(shootingPose,
                        gateClearControlPoint,
                        gateClearPose
                )).setConstantHeadingInterpolation(gateClearPose.getHeading())
                .setTValueConstraint(0.8)
                .build();

        shootGate2 = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(gatePickupPose,
                        shootingPose
                )).setLinearHeadingInterpolation(gatePickupPose.getHeading(), shootingPose.getHeading())
                .build();


        pickupClose = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(shootingPose, closePickupPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

//        clearGate = robot.getFollower().pathBuilder()
//                .addPath(new BezierCurve(closePickupPose, gateClearControlPoint,gateClearPose))
//                .setLinearHeadingInterpolation(closePickupPose.getHeading(), gateClearPose.getHeading())
//                .build();

        shootClose = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(closePickupPose, shootingPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

        pickupFar = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(shootingPose, farPickupControlPoint, farPickupPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

//        shootFar = robot.getFollower().pathBuilder()
//                .addPath(new BezierCurve(farPickupPose, farPickupControlPoint, parkPose))
//                .setConstantHeadingInterpolation(parkPose.getHeading())
//                .build();
        shootFar = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(farPickupPose, parkPose))
                .setConstantHeadingInterpolation(parkPose.getHeading())
                .build();

//        pickupCorner = robot.getFollower().pathBuilder()
//                .addPath(new BezierCurve(shootingPose, cornerControlPoint, cornerPose))
//                .setLinearHeadingInterpolation(shootingPose.getHeading(), cornerPose.getHeading())
//                .build();
//
//        backupCorner = robot.getFollower().pathBuilder()
//                .addPath(new BezierCurve(cornerPose, cornerBackupControlPoint, cornerPose))
//                .setConstantHeadingInterpolation(cornerPose.getHeading())
//                .build();
//
//        shootCorner = robot.getFollower().pathBuilder()
//                .addPath(new BezierCurve(cornerPose, cornerControlPoint, parkPose))
//                .setLinearHeadingInterpolation(cornerPose.getHeading(), parkPose.getHeading())
//                .build();
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
