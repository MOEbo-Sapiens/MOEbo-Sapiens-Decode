package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.hardware.ServoEx;

public class ProximityIndicator {
    private final ServoEx rgbLight;
    private final DistanceSensor distanceSensor;
    private final ElapsedTime pollTimer = new ElapsedTime();

    private static final double DETECTION_DISTANCE_CM = 4.0;
    private static final int MAX_OBJECTS = 3;

    private static final double COLOR_RED = 0.333;
    private static final double COLOR_ORANGE = 0.277;
    private static final double COLOR_YELLOW = 0.388;
    private static final double COLOR_GREEN = 0.5;

    private boolean wasObjectDetected = false;
    private int objectCount = 0;
    private double cachedDistanceCm = Double.NaN;
    private boolean cachedDetected = false;

    public ProximityIndicator(HardwareMap hardwareMap) {
        rgbLight = new ServoEx(hardwareMap, "rgbLight");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "color");
        pollTimer.reset();
    }

    public void update() {
        double pollIntervalMs = objectCount >= MAX_OBJECTS
                ? Constants.PROXIMITY_POLL_MS_FULL
                : Constants.PROXIMITY_POLL_MS;
        if (pollTimer.milliseconds() >= pollIntervalMs || Double.isNaN(cachedDistanceCm)) {
            cachedDistanceCm = distanceSensor.getDistance(DistanceUnit.CM);
            cachedDetected = cachedDistanceCm < DETECTION_DISTANCE_CM;
            pollTimer.reset();
        }

        boolean objectDetected = cachedDetected;

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

        rgbLight.setPosition(targetPosition);
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
        return rgbLight.getPosition();
    }
}
