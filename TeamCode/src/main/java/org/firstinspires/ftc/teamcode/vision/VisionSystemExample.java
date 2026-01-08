package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.pipelines.ArtifactColorPipeline;
import org.firstinspires.ftc.teamcode.vision.pipelines.LocalizationAprilTagPipeline;
import org.firstinspires.ftc.teamcode.vision.pipelines.ObeliskAprilTagPipeline;

/**
 * Example OpMode demonstrating the simplified vision system.
 */
@TeleOp(name = "Vision System Test", group = "Vision")
public class VisionSystemExample extends LinearOpMode {

    private LimelightManager vision;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize vision system
        vision = new LimelightManager(hardwareMap);

        // Register pipelines - pass manager reference so they can store data directly
        vision.registerPipeline(new ArtifactColorPipeline(0, vision));
        vision.registerPipeline(new ObeliskAprilTagPipeline(1, vision));
        vision.registerPipeline(new LocalizationAprilTagPipeline(2, vision));

        vision.setActivePipeline(0);

        telemetry.addLine("Vision System Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update all pipelines
            vision.updateAll();

            // Access artifact data directly from manager
            if (vision.hasTarget(0)) {
                telemetry.addLine("\n--- Artifacts ---");
                telemetry.addData("Sequence", String.join(" -> ", vision.artifactSequence));
                telemetry.addData("Confidence", vision.artifactConfidence + "%");

                // Make decisions based on color
                if (vision.artifactSequence.length > 0) {
                    if (vision.artifactSequence[0].equals("Red")) {
                        telemetry.addLine("→ Target red sample first");
                    }
                }
            }

            // Access obelisk data directly from manager
            if (vision.hasTarget(1)) {
                telemetry.addLine("\n--- Obelisk ---");
                telemetry.addData("Tag ID", vision.obeliskTagId);
                telemetry.addData("Angle", String.format("%.2f°", vision.obeliskTx));

                // Check alignment
                if (Math.abs(vision.obeliskTx) < 2.0) {
                    telemetry.addLine("✓ Aligned!");
                } else {
                    telemetry.addLine("Adjust by " + String.format("%.1f°", vision.obeliskTx));
                }
            }

            // Access localization data directly from manager
            if (vision.hasTarget(2)) {
                telemetry.addLine("\n--- Localization ---");
                telemetry.addData("Tags", vision.localizationTagIds.length);
                telemetry.addData("Position",
                        String.format("(%.1f, %.1f) @ %.1f°",
                                vision.localizationPoseX,
                                vision.localizationPoseY,
                                vision.localizationPoseHeading));

                // Check if reliable enough for odometry update
                if (vision.localizationConfidence > 0.8) {
                    telemetry.addLine("✓ Pose reliable - can update odometry");
                }
            }

            // Show all telemetry
            vision.addTelemetry(telemetry);
            telemetry.update();
        }
    }
}