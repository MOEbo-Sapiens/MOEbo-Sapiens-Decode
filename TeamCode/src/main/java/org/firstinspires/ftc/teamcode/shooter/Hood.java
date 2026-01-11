package org.firstinspires.ftc.teamcode.shooter;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hood {
    private CRServo hoodServo;
    public static double kP = 0, kD = 0, kF = 0;
    private PIDFController hoodPIDF = new PIDFController(new PIDFCoefficients(kP, 0, kD, kF));

    public Hood(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(CRServo.class, "Hood Servo");
    }


}
