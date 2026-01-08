package org.firstinspires.ftc.teamcode.vision.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.LimelightManager;

/**
 * Pipeline for detecting AprilTags on the Obelisk.
 * Results are stored directly in LimelightManager.
 */
public class ObeliskAprilTagPipeline extends BasePipeline {

    private final LimelightManager manager;

    public ObeliskAprilTagPipeline(int pipelineIndex, LimelightManager manager) {
        super(pipelineIndex, "ObeliskAprilTag");
        this.manager = manager;
    }

    @Override
    public void update() {
        // TODO: Replace with actual Limelight NetworkTables reading

        // Placeholder: Simulate detecting red obelisk tag
        hasTarget = true;

        if (hasTarget) {
            manager.obeliskTagId = 11; // Red obelisk
            manager.obeliskTx = 5.2;   // 5.2 degrees right
            manager.obeliskTy = -2.8;  // 2.8 degrees down
            manager.obeliskTa = 3.5;   // 3.5% of image
        } else {
            manager.obeliskTagId = -1;
            manager.obeliskTx = 0;
            manager.obeliskTy = 0;
            manager.obeliskTa = 0;
        }
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        super.addTelemetry(telemetry);
        if (hasTarget) {
            telemetry.addData(pipelineName + " Tag ID", manager.obeliskTagId);
            telemetry.addData(pipelineName + " TX", String.format("%.2f°", manager.obeliskTx));
            telemetry.addData(pipelineName + " TY", String.format("%.2f°", manager.obeliskTy));
        }
    }
}