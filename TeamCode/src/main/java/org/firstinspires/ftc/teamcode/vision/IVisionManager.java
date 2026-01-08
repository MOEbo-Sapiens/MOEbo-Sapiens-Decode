package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Generic vision manager interface.
 * Provides high-level vision operations independent of hardware implementation.
 */
public interface IVisionManager {

    void update();

    /**
     * Adds telemetry data
     */
    void addTelemetry(Telemetry telemetry);

    // Artifact/Sample Detection

    /**
     * Gets the detected artifact color sequence
     * @return array of color names (e.g. ["Red", "Blue", "Yellow"])
     */
    String[] getArtifactSequence();

    /**
     * Sets/overrides the artifact sequence (useful for testing or manual input)
     */
    void setArtifactSequence(String[] sequence);

    /**
     * Gets confidence of artifact detection
     * @return confidence percentage (0-100)
     */
    int getArtifactConfidence();

    /**
     * Checks if artifacts are detected
     */
    boolean hasArtifacts();

    // Obelisk Detection

    /**
     * Gets the obelisk tag ID
     * @return tag ID or -1 if not detected
     */
    int getObeliskTagId();

    /**
     * Determines which alliance the obelisk belongs to
     * @return "Red", "Blue", or "Unknown"
     */
    String determineObeliskAlliance();

    /**
     * Gets horizontal angle to obelisk
     * @return angle in degrees (negative = left, positive = right)
     */
    double getObeliskAngle();

    /**
     * Gets vertical angle to obelisk
     * @return angle in degrees
     */
    double getObeliskVerticalAngle();
    double getObeliskArea();
    boolean hasObelisk();
    Pose2D getLocalizationPose();
    void updateLocalizationIfReliable(LocalizationUpdateCallback updateCallback);
    double getLocalizationConfidence();

    int[] getLocalizationTagIds();

    boolean hasLocalization();

    interface LocalizationUpdateCallback {
        void updatePose(double x, double y, double heading);
    }
}