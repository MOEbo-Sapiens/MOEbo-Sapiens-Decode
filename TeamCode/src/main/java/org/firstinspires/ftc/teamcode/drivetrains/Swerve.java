package org.firstinspires.ftc.teamcode.drivetrains;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Swerve implements Drivetrain {
    Telemetry telemetry;
    private Follower follower;

    private static boolean fieldCentric = true;

    public Swerve(Follower follower, Telemetry telemetry) {
        this.follower = follower;
        this.telemetry = telemetry;
    }

    @Override
    public void update(Gamepad gamepad1, double speed, double rotSpeed) {
        arcade(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, speed, rotSpeed);
    }

    @Override
    public void arcade(double forward, double strafe, double rotate, double speed, double rotSpeed) {
        //[strafe, forward] [
        //
        if (fieldCentric) {
            double theta = -follower.getHeading();
            double cos = Math.cos(theta);
            double sin = Math.sin(theta);
            double strafeRot = strafe * cos - forward * sin;
            double forwardRot = strafe * sin + forward * cos;
            strafe = strafeRot;
            forward = forwardRot;
        }


        //pedro arcade has forward, left, cc as positive
        ((com.pedropathing.ftc.drivetrains.Swerve) follower.getDrivetrain()).arcadeDrive(-forward*speed, strafe*speed, -rotate*rotSpeed);
    }

    @Override
    public String name() {
        return "Swerve";
    }
}
