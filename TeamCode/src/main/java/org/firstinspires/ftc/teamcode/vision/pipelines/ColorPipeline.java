package org.firstinspires.ftc.teamcode.vision.pipelines;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline;
import org.firstinspires.ftc.teamcode.vision.VisionResult;

import java.util.List;

public class ColorPipeline implements VisionPipeline {

    private static final int PIPELINE_INDEX = 1;

    private final Telemetry telemetry;
    private Limelight3A limelight;
    private final VisionResult result = new VisionResult();

    public ColorPipeline(Telemetry telemetry) {
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
            List<LLResultTypes.ColorResult> colorResults = llResult.getColorResults();

            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Area",  llResult.getTa());
            telemetry.addData("Colors", colorResults.size());
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
        return "Color";
    }
}
