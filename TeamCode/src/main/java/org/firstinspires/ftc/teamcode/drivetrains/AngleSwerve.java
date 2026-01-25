package org.firstinspires.ftc.teamcode.drivetrains;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.VectorCalculator;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.util.AnglePID;
import org.firstinspires.ftc.teamcode.util.MathHelpers;
import org.firstinspires.ftc.teamcode.util.Vector2D;

@Config
public class AngleSwerve extends Swerve {
    public static double headingKP = 0.2;
    public static double headingKD = 100;

    double angle = 0;


    AnglePID anglePID = new AnglePID(new PIDCoefficients(headingKP, 0, headingKD));

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

        if (Constants.color == Constants.Color.BLUE) {
            driveVector.rotateVector(Math.toRadians(-90));
        } else {
            driveVector.rotateVector(Math.toRadians(90));
        }


        angle = (driveVector.getMagnitude() > 0.05) ? driveVector.getTheta() : angle;
        telemetry.addData("Joystick Angle", Math.toDegrees(angle));

        double headingPower = anglePID.calculate(angle, follower.getHeading());
        telemetry.addData("headingPower", headingPower);

        super.arcade(forward, strafe, headingPower,0, speed, rotSpeed);
    }

    @Override
    public String name() {
        return "Angle Swerve";
    }
}
