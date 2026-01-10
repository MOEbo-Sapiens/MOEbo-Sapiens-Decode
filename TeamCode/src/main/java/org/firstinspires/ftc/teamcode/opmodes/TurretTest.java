package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.vision.LimelightManager;
import org.firstinspires.ftc.teamcode.vision.Pipelines;
import org.firstinspires.ftc.teamcode.vision.VisionResult;
import org.firstinspires.ftc.teamcode.vision.pipelines.AprilTagPipeline;

@TeleOp
public class TurretTest extends LinearOpMode {
    private Turret turret;
    private LimelightManager limeLightManager;
    private AprilTagPipeline aprilTagPipeline;
    @Override
    public void runOpMode() throws InterruptedException {
        turret = new Turret(hardwareMap, telemetry);
        limeLightManager = new LimelightManager(hardwareMap, telemetry);
        limeLightManager.setPipeline(Pipelines.APRIL_TAG);

        waitForStart();
        while(opModeIsActive()) {
            limeLightManager.process(null);

            VisionResult result = limeLightManager.getResult();
            double tx = result.pose.getPosition().x; // REALLLY HACKKY but if it works it works yk

            turret.turnToAngle(tx);
            telemetry.update();
        }
    }
}
