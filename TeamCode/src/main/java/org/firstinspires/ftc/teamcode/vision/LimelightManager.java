package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.pipelines.IVisionPipeline;

import java.util.HashMap;
import java.util.Map;

public class LimelightManager {

    private final Map<Integer, IVisionPipeline> pipelines;
    private int activePipelineIndex;

    public String[] artifactSequence = new String[0];
    public int artifactConfidence = 0;

    public int obeliskTagId = -1;
    public double obeliskTx = 0;
    public double obeliskTy = 0;
    public double obeliskTa = 0;

    public int[] localizationTagIds = new int[0];
    public double localizationPoseX = 0;
    public double localizationPoseY = 0;
    public double localizationPoseHeading = 0;
    public double localizationConfidence = 0;

    //Add actual Limelight hardware interface here

    public LimelightManager(HardwareMap hardwareMap) {
        this.pipelines = new HashMap<>();
        this.activePipelineIndex = 0;

        // Initialize actual Limelight connection
        // this.limelight = hardwareMap.get(LimelightDevice.class, "limelight");
    }

    public void updateAll() {
        for (IVisionPipeline pipeline : pipelines.values()) {
            pipeline.update();
        }
    }

    public void registerPipeline(IVisionPipeline pipeline) {
        pipelines.put(pipeline.getPipelineIndex(), pipeline);
    }

    public void setActivePipeline(int pipelineIndex) {
        this.activePipelineIndex = pipelineIndex;
        //Send pipeline switch command to Limelight

        /* PIPELINES:
           0 - MegaTag2 / AprilTag Localization
           1 - Goal April Tags (maybe unneeded)
           2 - Green Artifact Color Detection
           3 - Neutral Artifact Detection
           4 - Purple Artifact Color Detection
           5 - Obelisk April Tags
           6/7+ - Empty
        */
    }

    public int getActivePipelineIndex() {
        return activePipelineIndex;
    }

    public boolean hasTarget(int pipelineIndex) {
        IVisionPipeline pipeline = pipelines.get(pipelineIndex);
        return pipeline != null && pipeline.hasTarget();
    }
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Active Pipeline", activePipelineIndex);
        for (IVisionPipeline pipeline : pipelines.values()) {
            pipeline.addTelemetry(telemetry);
        }
    }
}