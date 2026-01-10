package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.robot.Robot;

public interface VisionPipeline {

    void initialize(Limelight3A limelight);

    void process(Robot robot);

    VisionResult getResult();

    int getPipelineIndex();

    String name();
}
