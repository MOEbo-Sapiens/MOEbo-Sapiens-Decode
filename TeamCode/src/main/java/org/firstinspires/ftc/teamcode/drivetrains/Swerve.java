package org.firstinspires.ftc.teamcode.drivetrains;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Swerve implements Drivetrain {
   Telemetry telemetry;
    private com.pedropathing.ftc.drivetrains.Swerve dt;

    public Swerve(Follower follower, Telemetry telemetry) {
       dt = (com.pedropathing.ftc.drivetrains.Swerve) follower.getDrivetrain();
       this.telemetry = telemetry;
    }

    @Override
    public void update(Gamepad gamepad1, double speed, double rotSpeed) {
        arcade(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, speed, rotSpeed);
    }

    @Override
    public void arcade(double forward, double strafe, double rotate, double speed, double rotSpeed) {
        //pedro arcade has forward, left, cc as positive
        //our arcade is intended for forward, right, clockwise as positive
        dt.arcadeDrive(forward*speed, -strafe*speed, -rotate*rotSpeed);
    }

    @Override
    public String name() {
        return "Swerve";
    }
}
