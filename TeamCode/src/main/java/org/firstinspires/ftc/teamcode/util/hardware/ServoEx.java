package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Constants;

public class ServoEx {
    private final Servo servo;
    private double cachingTolerance = Constants.SERVO_CACHING_TOLERANCE;
    private double lastPosition = Double.NaN;

    public ServoEx(HardwareMap hardwareMap, String name) {
        servo = hardwareMap.get(Servo.class, name);
    }

    public ServoEx setCachingTolerance(double cachingTolerance) {
        this.cachingTolerance = cachingTolerance;
        return this;
    }

    public double getCachingTolerance() {
        return cachingTolerance;
    }

    public void setPosition(double position) {
        if (Double.isNaN(lastPosition) || Math.abs(position - lastPosition) > cachingTolerance) {
            servo.setPosition(position);
            lastPosition = position;
        }
    }

    public double getPosition() {
        return Double.isNaN(lastPosition) ? servo.getPosition() : lastPosition;
    }

    public void setDirection(Servo.Direction direction) {
        servo.setDirection(direction);
    }

    public Servo getServo() {
        return servo;
    }
}
