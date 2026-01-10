package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@FunctionalInterface
public interface VisionPipelineSupplier {
    VisionPipeline get(Telemetry telemetry);
}
