package org.firstinspires.ftc.teamcode.vision.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.LimelightManager;

/**
 * Pipeline for detecting the color sequence of game artifacts.
 * Results are stored directly in LimelightManager.
 */
public class ArtifactColorPipeline extends BasePipeline {

    private final LimelightManager manager;

    public ArtifactColorPipeline(int pipelineIndex, LimelightManager manager) {
        super(pipelineIndex, "ArtifactColor");
        this.manager = manager;
    }

    @Override
    public void update() {
        // TODO: Replace with actual Limelight NetworkTables reading

        // Placeholder: Simulate detecting 3 samples (Red, Blue, Yellow)
        hasTarget = true;

        if (hasTarget) {
            // Store directly in manager
            manager.artifactSequence = new String[]{"Red", "Blue", "Yellow"};
            manager.artifactConfidence = 85;
        } else {
            manager.artifactSequence = new String[0];
            manager.artifactConfidence = 0;
        }
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        super.addTelemetry(telemetry);
        if (hasTarget) {
            telemetry.addData(pipelineName + " Sequence", String.join(" -> ", manager.artifactSequence));
            telemetry.addData(pipelineName + " Confidence", manager.artifactConfidence + "%");
        }
    }
}