package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CRServoEx {
    private final CRServo servo;
    private double cachingTolerance = 0.0001;
    private double lastPower = Double.NaN;

    public CRServoEx(HardwareMap hardwareMap, String name) {
        servo = hardwareMap.get(CRServo.class, name);
    }

    public CRServoEx setCachingTolerance(double cachingTolerance) {
        this.cachingTolerance = cachingTolerance;
        return this;
    }

    public double getCachingTolerance() {
        return cachingTolerance;
    }

    public void setPower(double power) {
        if (Double.isNaN(lastPower) || Math.abs(power - lastPower) > cachingTolerance
                || (power == 0 && lastPower != 0)) {
            servo.setPower(power);
            lastPower = power;
        }
    }

    public double getPower() {
        return servo.getPower();
    }

    public void setDirection(CRServo.Direction direction) {
        servo.setDirection(direction);
    }

    public CRServo getServo() {
        return servo;
    }
}
