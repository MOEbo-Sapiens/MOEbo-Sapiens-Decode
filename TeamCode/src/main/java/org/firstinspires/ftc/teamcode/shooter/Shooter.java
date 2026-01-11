package org.firstinspires.ftc.teamcode.shooter;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    private DcMotorEx shooterMotor;
    private CRServo hoodServo;
    private Servo gateServo;

    private PIDFCoefficients

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter Motor");
        hoodServo = hardwareMap.get(CRServo.class, "Hood Servo");
        gateServo = hardwareMap.get(Servo.class, "Gate Servo");
    }

    setPower((kV * getTarget()) + (kP * (getTarget() - getVelocity())) + kS);



}
