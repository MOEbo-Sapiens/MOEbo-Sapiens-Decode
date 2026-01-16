package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.CoaxialPod;
import com.pedropathing.ftc.drivetrains.SwerveConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PedroConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
        .forwardZeroPowerAcceleration(-138.72)
        .lateralZeroPowerAcceleration(-138.72)
        .useSecondaryDrivePIDF(true)
        .useSecondaryHeadingPIDF(true)
        .useSecondaryTranslationalPIDF(true)
        .translationalPIDFCoefficients(new PIDFCoefficients(0.125, 0, 0.008 , 0))
        .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.0825, 0, 0.008, 0.01))
        .headingPIDFCoefficients(new PIDFCoefficients(1, 0 , 0.003, 0))
        .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.63, 0, 0.035, 0))
        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0035, 0, 0.00001, 0.6, 0.13))
        .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0, 0.000005, 0.6, 0.13))
        .centripetalScaling(0.002)
        .mass(5.362); //TODO: actually weigh the robot, in kg

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(6.4390354331) // 163.5515 mm
            .strafePodX(-0.0749409449) // -1.9035 mm
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static SwerveConstants swerveConstants = new SwerveConstants()
            .velocity(77.45)
            .useBrakeModeInTeleOp(true);

    private static CoaxialPod leftFront(HardwareMap hardwareMap) {
        return new CoaxialPod(
                hardwareMap,
                "sm2", "ss2", "se2",
                new PIDFCoefficients(0.005, 0, 0.0, 0.0),
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD,
                -81.38, new Pose(305.86624, 311.4),
                0, 3.3,
                false
        );
    }

    private static CoaxialPod rightFront(HardwareMap hardwareMap) {
        return new CoaxialPod(
                hardwareMap,
                "sm1", "ss1", "se1",
                new PIDFCoefficients(0.005, 0.0, 0.0, 0.0),
                DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD,
                -11.0182, new Pose(305.86624, -311.4),
                0, 3.3,
                false
        );
    }

    private static CoaxialPod leftBack(HardwareMap hardwareMap) {
        return new CoaxialPod(
                hardwareMap,
                "sm3", "ss3", "se3",
                new PIDFCoefficients(0.005, 0.0, 0.0, 0.0),
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD,
                34.16, new Pose(-305.86624, 311.4),
                0, 3.3,
                false
        );
    }

    private static CoaxialPod rightBack(HardwareMap hardwareMap) {
        return new CoaxialPod(
                hardwareMap,
                "sm0", "ss0", "se0",
                new PIDFCoefficients(0.005, 0.0, 0.0, 0.0),
                DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD,
                -71.6727, new Pose(-305.86624, -311.4),
                0, 3.3,
                false
        );
    }

    public static PathConstraints pathConstraints = new PathConstraints(0.95, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .swerveDrivetrain(swerveConstants, leftFront(hardwareMap), rightFront(hardwareMap), leftBack(hardwareMap), rightBack(hardwareMap))
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
