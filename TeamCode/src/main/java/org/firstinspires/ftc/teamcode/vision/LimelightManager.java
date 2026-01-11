package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class LimelightManager {

    private final Limelight3A limelight;
    private final Telemetry telemetry;

    private VisionPipeline currentPipeline;
    private Pipelines currentPipelineEnum;
    private int pipelineIndex;

    public LimelightManager(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);
        limelight.start();

        pipelineIndex = 0;
    }

    public void setPipeline(Pipelines pipeline) {
        currentPipelineEnum = pipeline;
        currentPipeline = pipeline.build(telemetry);

        if (currentPipeline != null) {
            currentPipeline.initialize(limelight);
        }
    }

    public void process(Robot robot) {
        if (currentPipeline != null) {
            currentPipeline.process(robot);
        }
    }

    public void cycleNext() {
        Pipelines[] pipelines = Pipelines.getActivePipelines();
        pipelineIndex = (pipelineIndex + 1) % pipelines.length;
        setPipeline(pipelines[pipelineIndex]);
    }

    public void cyclePrevious() {
        Pipelines[] pipelines = Pipelines.getActivePipelines();
        pipelineIndex = (pipelineIndex - 1 + pipelines.length) % pipelines.length;
        setPipeline(pipelines[pipelineIndex]);
    }

    public Pipelines getCurrentPipelineEnum() {
        return currentPipelineEnum != null ? currentPipelineEnum : Pipelines.NONE;
    }

    public VisionPipeline getCurrentPipeline() {
        return currentPipeline;
    }

    public VisionResult getResult() {
        return currentPipeline != null ? currentPipeline.getResult() : new VisionResult();
    }

    public String getCurrentPipelineName() {
        return currentPipeline != null ? currentPipeline.name() : "None";
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    public void stop() {
        limelight.stop();
    }

    public void start() {
        limelight.start();
    }
}
