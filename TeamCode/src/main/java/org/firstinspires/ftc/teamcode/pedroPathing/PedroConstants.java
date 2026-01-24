package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
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
        .forwardZeroPowerAcceleration(-97.41126)
        .lateralZeroPowerAcceleration(-97.41126)
        .useSecondaryDrivePIDF(true)
        .useSecondaryHeadingPIDF(true)
        .useSecondaryTranslationalPIDF(true)

        .translationalPIDFCoefficients(new PIDFCoefficients(0.125, 0, 0.008 , 0))
        .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.0825, 0, 0.008, 0))

        .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0 , 0.003, 0))
        .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.75, 0, 0.03, 0))

        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0, 0.00003, 0.6, 0.13))
        .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.004, 0, 0.000002, 0.6, 0.13))


//        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0,0,0,0,0))
//        .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0,0,0,0,0))

//        .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(
//                0.05, //0.05 to 0.3
//                0,//0.38735914623969386,
//                0.002)
//        )
        .centripetalScaling(0.002)
        .mass(5.362); //TODO: actually weigh the robot, in kg

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(6.4390354331) // 163.5515 mm
            .strafePodX(-0.0749409449) // -1.9035 mm
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static SwerveConstants swerveConstants = new SwerveConstants()
            .velocity(83.93)
            .useBrakeModeInTeleOp(true);

   // F - front: .130, back: .190
    // P=0.00645 D=0.00019
    // P=0.00549 D=0.00028

    private static double kP = 0.0055;
    private static double kD = 0.00015;
    private static double kFFront = 0.0130;
    private static double kFBack = 0.0190;

    private static CoaxialPod leftFront(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(
                hardwareMap,
                "sm2", "ss2", "se2",
                new PIDFCoefficients(kP, 0, kD, kFFront),
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD,
                275.1546707504, new Pose(305.86624, 311.4),
                0.025, 3.290,
                false
        );
        return pod;
    }

    private static CoaxialPod rightFront(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(
                hardwareMap,
                "sm1", "ss1", "se1",
                new PIDFCoefficients(kP, 0, kD, kFFront),
                DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD,
                346.56880733944956, new Pose(305.86624, -311.4),
                0.018, 3.288,
                false
        );
        return pod;
    }

    private static CoaxialPod leftBack(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(
                hardwareMap,
                "sm3", "ss3", "se3",
                new PIDFCoefficients(kP, 0, kD, kFBack),
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD,
                34.1549725442, new Pose(-305.86624, 311.4),
                0.029, 3.307,
                false
        );
        return pod;
    }

    private static CoaxialPod rightBack(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(
                hardwareMap,
                "sm0", "ss0", "se0",
                new PIDFCoefficients(kP, 0, kD, kFBack),
                DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD,
                288.7557042896, new Pose(-305.86624, -311.4),
                0.014, 3.301,
                false
        );
        return pod;
    }

    //TODO: TUNE THESE, CAN MAKE A HUGE DIFF
    public static PathConstraints pathConstraints = new PathConstraints(0.95, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .swerveDrivetrain(swerveConstants, leftFront(hardwareMap), rightFront(hardwareMap), leftBack(hardwareMap), rightBack(hardwareMap))
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
