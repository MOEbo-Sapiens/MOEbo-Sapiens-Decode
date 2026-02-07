package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.robot.Constants;

public abstract class CompatabilityAuto extends Auto{

    @Override
    protected void createAutoCommands() {
//        robot.getFollower().setMaxPower(0.9);
        updateShooter = robot.updateShootingSubsystems();
//        updateTurret = robot.updateTurret();

        double shootTime = 610;

        schedule(
                infinite(() -> telemetry.addData("Last Turret Ticks", Constants.getLastTurretTicks())),
                sequential(
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
//                                race(
//                                        turnTo(robot.getFollower(), shootingPose.getHeading()),
//                                        waitMs(shootTime)
//                                ),
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
//                                race(
//                                        turnTo(robot.getFollower(), gateClearPose.getHeading()),
//                                        waitMs(shootTime)
//                                ),
//                                instant(() -> robot.getFollower().breakFollowing()),
                                        follow(robot.getFollower(), clearGate)
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
                        waitMs(400),

                        parallel(
                                follow(robot.getFollower(), shootGate),
                                sequential(
                                        waitMs(500),
                                        setShooting()
                                )
                        ),
//                waitMs(200),
//                waitMs(750),

//                waitMs(200),
                        //second gate clear / pickup + shoot
                        parallel(
                                shootAndSetIntaking(),
                                //second gate clear / pickup + shoot
                                sequential(
                                        waitMs(shootTime),
//                                race(
//                                        turnTo(robot.getFollower(), gateClearPose.getHeading()),
//                                        waitMs(shootTime)
//                                ),
                                        follow(robot.getFollower(), clearGate)
//                                parallel(
//                                        follow(robot.getFollower(), clearGate1),
//                                        sequential(
//                                                waitUntil(() -> robot.getFollower().getCurrentTValue() > 0.85),
//                                                instant(() -> robot.getFollower().breakFollowing())
//                                        )
//                                )
                                )
                        ),
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
                        waitMs(400),

                        parallel(
                                follow(robot.getFollower(), shootGate),
                                sequential(
                                        waitMs(500),
                                        setShooting()
                                )
                        ),


                        //third gate clear / pickup + shoot
                        parallel(
                                shootAndSetIntaking(),
                                //third gate clear / pickup + shoot
                                sequential(
                                        waitMs(shootTime),
//                                race(
//                                        turnTo(robot.getFollower(), gateClearPose.getHeading()),
//                                        waitMs(shootTime)
//                                ),
                                        follow(robot.getFollower(), clearGate)
//                                parallel(
//                                        follow(robot.getFollower(), clearGate1),
//                                        sequential(
//                                                waitUntil(() -> robot.getFollower().getCurrentTValue() > 0.85),
//                                                instant(() -> robot.getFollower().breakFollowing())
//                                        )
//                                )
                                )
                        ),
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
                        waitMs(400),

                        parallel(
                                follow(robot.getFollower(), shootGate),
                                sequential(
                                        waitMs(500),
                                        setShooting()
                                )
                        ),

//                waitMs(200),
                        parallel(
                                shootAndSetIntaking(),
                                //shoot and pickup close
                                sequential(
                                        waitMs(shootTime),
//                                race(
//                                        turnTo(robot.getFollower(), shootingPose.getHeading()),
//                                        waitMs(shootTime)
//                                ),
                                        parallel(
                                                follow(robot.getFollower(), pickupClose),
                                                sequential(
                                                        waitMs(750),
                                                        robot.setIntakePower(1)
                                                )
                                        )
                                )
                        ),
//                waitMs(300),


                        parallel(
                                follow(robot.getFollower(), shootClose),
                                sequential(
                                        waitMs(200),
                                        setShooting()
                                )
                        ),



                        shootAndSetIntaking(),

                        robot.setIntakePower(0)
//                        robot.setTurretPos(0)
                ));
    }

}
