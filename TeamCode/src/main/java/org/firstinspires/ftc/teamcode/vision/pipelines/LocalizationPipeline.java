package org.firstinspires.ftc.teamcode.vision.pipelines;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline;
import org.firstinspires.ftc.teamcode.vision.VisionResult;

public class LocalizationPipeline implements VisionPipeline {

    private static final int PIPELINE_INDEX = 2;

    private final Telemetry telemetry;
    private Limelight3A limelight;
    private VisionResult result = new VisionResult();

    public LocalizationPipeline(Telemetry telemetry) {
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
            Pose3D pose = llResult.getBotpose_MT2();
            int tagCount = llResult.getFiducialResults().size();
            result = new VisionResult(pose, tagCount);

            if (pose != null) {
                telemetry.addData("MT2 X", "%.2f", pose.getPosition().x);
                telemetry.addData("MT2 Y", "%.2f", pose.getPosition().y);
                telemetry.addData("MT2 Yaw", "%.1f°", pose.getOrientation().getYaw());
                telemetry.addData("Tags", tagCount);
            }
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
        return "Localization";
    }
}
