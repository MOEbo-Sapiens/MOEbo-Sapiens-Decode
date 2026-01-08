package org.firstinspires.ftc.teamcode.vision.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Abstract base class for vision pipelines.
 */
public abstract class BasePipeline implements IVisionPipeline {

    protected final int pipelineIndex;
    protected final String pipelineName;
    protected boolean hasTarget = false;

    protected BasePipeline(int pipelineIndex, String pipelineName) {
        this.pipelineIndex = pipelineIndex;
        this.pipelineName = pipelineName;
    }

    @Override
    public int getPipelineIndex() {
        return pipelineIndex;
    }

    @Override
    public String getPipelineName() {
        return pipelineName;
    }

    @Override
    public boolean hasTarget() {
        return hasTarget;
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData(pipelineName + " Target", hasTarget ? "Yes" : "No");
    }
}