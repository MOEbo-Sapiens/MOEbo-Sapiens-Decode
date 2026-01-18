package org.firstinspires.ftc.teamcode.drivetrains;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.util.AnglePID;
import org.firstinspires.ftc.teamcode.util.Vector2D;

public class AngleSwerve extends Swerve {
    public static double headingKP = -0.1;
    public static double headingKD = -0.005;

    AnglePID basicPID = new AnglePID(new PIDCoefficients(headingKP, 0, headingKD));

    //AnglePID
    public AngleSwerve(Follower follower, Telemetry telemetry) {
        super(follower, telemetry);
    }

    @Override
    public void update(Gamepad gamepad1, double speed, double rotSpeed) {
        super.update(gamepad1, speed, rotSpeed);
    }

    @Override
    public void arcade(double forward, double strafe, double rotateX, double rotateY, double speed, double rotSpeed) {
        Vector2D driveVector = new Vector2D(rotateX, rotateY);

        if (Constants.color == Constants.Color.RED) {
            driveVector.rotateVector(Math.toRadians(-90));
        } else {
            driveVector.rotateVector(Math.toRadians(90));
        }

        double joystickAngle = driveVector.getTheta();

        telemetry.addData("Joystick Angle", Math.toDegrees(joystickAngle));

        double headingPower = basicPID.calculate(joystickAngle, follower.getHeading());

        telemetry.addData("headingPower", headingPower);

        super.arcade(forward, strafe, headingPower,0, speed, rotSpeed);
    }

    @Override
    public String name() {
        return "Angle Swerve";
    }
}
