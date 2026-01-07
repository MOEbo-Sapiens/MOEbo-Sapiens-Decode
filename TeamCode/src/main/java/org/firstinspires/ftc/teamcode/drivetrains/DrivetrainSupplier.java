package org.firstinspires.ftc.teamcode.drivetrains;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@FunctionalInterface
public interface DrivetrainSupplier {
    Drivetrain get(Follower follower, Telemetry telemetry);
}
