package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.TestTurret;
import org.firstinspires.ftc.teamcode.vision.LimelightManager;
import org.firstinspires.ftc.teamcode.vision.Pipelines;
import org.firstinspires.ftc.teamcode.vision.VisionResult;
import org.firstinspires.ftc.teamcode.vision.pipelines.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.util.telemetry.FastTelemetry;

@Disabled
@TeleOp
public class TurretTestOld extends LinearOpMode {
    private TestTurret testTurret;
    private LimelightManager limeLightManager;
    private AprilTagPipeline aprilTagPipeline;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new FastTelemetry(telemetry);
        testTurret = new TestTurret(hardwareMap, telemetry);
        limeLightManager = new LimelightManager(hardwareMap, telemetry);
        limeLightManager.setPipeline(Pipelines.APRIL_TAG);

        waitForStart();
        while(opModeIsActive()) {
            limeLightManager.process(null);

            VisionResult result = limeLightManager.getResult();
            double tx = result.pose.getPosition().x; // REALLLY HACKKY but if it works it works yk

            testTurret.turnToAngle(tx);
            telemetry.update();
        }
    }
}
