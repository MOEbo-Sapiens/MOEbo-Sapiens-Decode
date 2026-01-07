package org.firstinspires.ftc.teamcode.drivetrains;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AngleMecanum extends Mecanum{
    //AnglePID

    public AngleMecanum(Follower follower, Telemetry telemetry) {
        super(follower, telemetry);
    }

    @Override
    public void update(Gamepad gamepad1, double speed, double rotSpeed) {
        //TODO:    implement
        super.update(gamepad1, speed, rotSpeed);
    }

    @Override
    public void arcade(double forward, double strafe, double rotate, double speed, double rotSpeed) {
        //TODO:    implement
        super.arcade(forward, strafe, rotate, speed, rotSpeed);
    }

    @Override
    public String name() {
        return "Angle Mecanum";
    }
}
