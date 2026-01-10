package org.firstinspires.ftc.teamcode.vision.pipelines;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline;
import org.firstinspires.ftc.teamcode.vision.VisionResult;

public class AprilTagPipeline implements VisionPipeline {

    private static final int PIPELINE_INDEX = 0;

    private final Telemetry telemetry;
    private Limelight3A limelight;
    private VisionResult result = new VisionResult();

    public AprilTagPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void initialize(Limelight3A limelight) {
        this.limelight = limelight;
        limelight.pipelineSwitch(PIPELINE_INDEX);
    }

    @Override
    public void process(Robot robot) {
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            Pose3D pose = llResult.getBotpose();
            int tagCount = llResult.getFiducialResults().size();
            result = new VisionResult(pose, tagCount);

            telemetry.addData("Tx", "%.1f", llResult.getTx());
            telemetry.addData("Ty", "%.1f", llResult.getTy());
            telemetry.addData("Tags", tagCount);
        }
    }

    @Override
    public VisionResult getResult() {
        return result;
    }

    @Override
    public int getPipelineIndex() {
        return PIPELINE_INDEX;
    }

    @Override
    public String name() {
        return "AprilTag";
    }
}
