package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Constants;

import smile.interpolation.LinearInterpolation;

public class Hood {
    private Servo hoodServo;
    boolean activated = false;

    public Hood(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "Hood Servo");
    }

    private double[] servoPositions = new double[] {0, 1}; //TODO: actually tune
    private double[] hoodAngles = new double[] {Constants.MIN_HOOD_ANGLE, Constants.MAX_HOOD_ANGLE};
    private double[] launchAngles = new double[] {Constants.MAX_LAUNCH_ANGLE, Constants.MIN_LAUNCH_ANGLE};

    LinearInterpolation hoodAngleToServo = new LinearInterpolation(hoodAngles, servoPositions);
    LinearInterpolation servoToHoodAngle = new LinearInterpolation(servoPositions, hoodAngles);

    LinearInterpolation launchAngleToServo = new LinearInterpolation(launchAngles, servoPositions);
    LinearInterpolation servoToLaunchAngle = new LinearInterpolation(servoPositions, launchAngles);

    private void setTargetPosition(double position) {
        hoodServo.setPosition(position);
    }

    private double getCurrentPosition() {
        return hoodServo.getPosition();
    }


    public void setHoodAngle(double radians) {
        setTargetPosition(
            hoodAngleToServo.interpolate(radians)
        );
    }

    public void setLaunchAngle(double radians) {
        setTargetPosition(
            launchAngleToServo.interpolate(radians)
        );
    }

    public double getLaunchAngle() {
       return servoToLaunchAngle.interpolate(getCurrentPosition());
    }

    public double getHoodAngle() {
        return servoToHoodAngle.interpolate(getCurrentPosition());
    }
}
