package org.firstinspires.ftc.teamcode.vision.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface IVisionPipeline {

    void update();

    int getPipelineIndex();

    String getPipelineName();

    boolean hasTarget();

    void addTelemetry(Telemetry telemetry);
}