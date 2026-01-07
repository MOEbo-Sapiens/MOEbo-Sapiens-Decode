package org.firstinspires.ftc.teamcode.robot;


import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Drivetrain {
    default void update(Gamepad gamepad) {
        update(gamepad, 1.0, 1.0);
    }

    //no gamepad 2 bc not needed
    void update(Gamepad gamepad1, double speed, double rotSpeed);


    default void arcade (double forward, double strafe, double rotate) {
        arcade(forward, strafe, rotate, 1.0, 1.0);
    }

    void arcade(double forward, double strafe, double rotate, double speed, double rotSpeed);

    public enum DriveMode {
        SWERVE,
        SWERVE_ANGLE,
        MECANUM,
        MECANUM_ANGLE
    }
}
