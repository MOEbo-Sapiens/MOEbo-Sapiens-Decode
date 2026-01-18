package org.firstinspires.ftc.teamcode.drivetrains;


import com.qualcomm.robotcore.hardware.Gamepad;

public interface Drivetrain {
    default void update(Gamepad gamepad) {
        update(gamepad, 1.0, 1.0);
    }

    //no gamepad 2 bc not needed
    void update(Gamepad gamepad1, double speed, double rotSpeed);


    default void arcade (double forward, double strafe, double rotateX, double rotateY) {
        arcade(forward, strafe, rotateX, rotateY, 1.0, 1.0);
    }

    void arcade(double forward, double strafe, double rotateX, double rotateY, double speed, double rotSpeed);

    String name();
}
