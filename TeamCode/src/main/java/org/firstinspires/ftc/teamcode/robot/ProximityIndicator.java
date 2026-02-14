package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ProximityIndicator {
    private final Servo rgbLight;
    private final DistanceSensor distanceSensor;

    private static final double DETECTION_DISTANCE_CM = 4.0;
    private static final int MAX_OBJECTS = 3;

    private static final double COLOR_RED = 0.333;
    private static final double COLOR_ORANGE = 0.277;
    private static final double COLOR_YELLOW = 0.388;
    private static final double COLOR_GREEN = 0.5;

    private double cachedPosition = -1;
    private boolean wasObjectDetected = false;
    private int objectCount = 0;

    public ProximityIndicator(HardwareMap hardwareMap) {
        rgbLight = hardwareMap.get(Servo.class, "rgbLight");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "color");
    }

    public void update() {
        boolean objectDetected = distanceSensor.getDistance(DistanceUnit.CM) < DETECTION_DISTANCE_CM;

        if (objectDetected && !wasObjectDetected && objectCount < MAX_OBJECTS) {
            objectCount++;
        }

        if (objectCount >= MAX_OBJECTS && !objectDetected) {
            objectCount = 0;
        }

        wasObjectDetected = objectDetected;

        double targetPosition;
        if (objectCount >= MAX_OBJECTS) {
            targetPosition = COLOR_GREEN;
        } else if (objectCount == 2) {
            targetPosition = COLOR_YELLOW;
        } else if (objectCount == 1) {
            targetPosition = COLOR_ORANGE;
        } else {
            targetPosition = COLOR_RED;
        }

        if (targetPosition != cachedPosition) {
            cachedPosition = targetPosition;
            rgbLight.setPosition(targetPosition);
        }
    }

    public int getObjectCount() {
        return objectCount;
    }

    public boolean isFull() {
        return objectCount >= MAX_OBJECTS;
    }

    public void resetCount() {
        objectCount = 0;
    }

    public double getColorPosition() {
        return cachedPosition;
    }
}
