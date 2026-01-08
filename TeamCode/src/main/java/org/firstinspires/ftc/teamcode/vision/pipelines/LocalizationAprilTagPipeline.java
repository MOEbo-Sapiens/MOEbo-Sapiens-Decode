package org.firstinspires.ftc.teamcode.vision.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.LimelightManager;

/**
 * Pipeline for detecting field localization AprilTags.
 * Results are stored directly in LimelightManager.
 */
public class LocalizationAprilTagPipeline extends BasePipeline {

    private final LimelightManager manager;

    public LocalizationAprilTagPipeline(int pipelineIndex, LimelightManager manager) {
        super(pipelineIndex, "LocalizationAprilTag");
        this.manager = manager;
    }

    @Override
    public void update() {
        // TODO: Replace with actual Limelight NetworkTables reading
        // Read from Limelight's MegaTag or AprilTag pipeline

        // Placeholder: Simulate detecting 2 field tags
        hasTarget = true;

        if (hasTarget) {
            manager.localizationTagIds = new int[]{1, 2};
            manager.localizationPoseX = 48.0;      // inches
            manager.localizationPoseY = 48.0;      // inches
            manager.localizationPoseHeading = 90.0; // degrees
            manager.localizationConfidence = 0.85;  // 85% confidence
        } else {
            manager.localizationTagIds = new int[0];
            manager.localizationPoseX = 0;
            manager.localizationPoseY = 0;
            manager.localizationPoseHeading = 0;
            manager.localizationConfidence = 0;
        }
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        super.addTelemetry(telemetry);
        if (hasTarget) {
            telemetry.addData(pipelineName + " Tags", manager.localizationTagIds.length);
            telemetry.addData(pipelineName + " X", String.format("%.2f in", manager.localizationPoseX));
            telemetry.addData(pipelineName + " Y", String.format("%.2f in", manager.localizationPoseY));
            telemetry.addData(pipelineName + " Heading", String.format("%.2f°", manager.localizationPoseHeading));
            telemetry.addData(pipelineName + " Confidence", String.format("%.1f%%", manager.localizationConfidence * 100));
        }
    }
}