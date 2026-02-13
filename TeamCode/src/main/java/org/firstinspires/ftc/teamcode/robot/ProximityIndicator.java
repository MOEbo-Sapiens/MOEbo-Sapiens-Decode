package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ProximityIndicator {
    private final RevBlinkinLedDriver blinkin;
    private final DistanceSensor distanceSensor;

    private static final double DETECTION_DISTANCE_CM = 4.0;
    private static final int MAX_OBJECTS = 3;

    private BlinkinPattern cachedPattern = null;
    private boolean wasObjectDetected = false;
    private int objectCount = 0;

    public ProximityIndicator(HardwareMap hardwareMap) {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "color");
    }

    public void update() {
        boolean objectDetected = distanceSensor.getDistance(DistanceUnit.CM) < DETECTION_DISTANCE_CM;

        // Detect rising edge (object just entered)
        if (objectDetected && !wasObjectDetected && objectCount < MAX_OBJECTS) {
            objectCount++;
        }

        // Reset when full and object leaves
        if (objectCount >= MAX_OBJECTS && !objectDetected) {
            objectCount = 0;
        }

        wasObjectDetected = objectDetected;

        BlinkinPattern targetPattern;
        if (objectCount >= MAX_OBJECTS) {
            targetPattern = BlinkinPattern.GREEN;
        } else if (objectCount == 2) {
            targetPattern = BlinkinPattern.YELLOW;
        } else if (objectCount == 1) {
            targetPattern = BlinkinPattern.ORANGE;
        } else {
            targetPattern = BlinkinPattern.RED;
        }

        if (targetPattern != cachedPattern) {
            cachedPattern = targetPattern;
            blinkin.setPattern(targetPattern);
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
}
