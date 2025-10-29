package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(6.3)
            .forwardZeroPowerAcceleration(-41.05)
            .lateralZeroPowerAcceleration(-71.74)

            .useSecondaryHeadingPIDF(true)
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1.3,
                    0,
                    0.05,
                    0.002))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    1.5,
                    0,
                    0.09,
                    0.002))

            .useSecondaryTranslationalPIDF(false)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.1,
                    0,
                    0.008,
                    0.002))
            //.secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.12,0,0.001,0.013))

            //.useSecondaryDrivePIDF(false)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.004,
                    0.0,
                    0.001,
                    0.6,
                    0.002))

            //.centripetalScaling(0.001)
            ;


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(88.000)
            .yVelocity(78.000)
            ;


    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(.001980)
            .strafeTicksToInches(.002030)
            .turnTicksToInches(.001980)
            .leftPodY(3.34)
            .rightPodY(-3.34)
            .strafePodX(3.34)
            .leftEncoder_HardwareMapName("lr")
            .rightEncoder_HardwareMapName("rf")
            .strafeEncoder_HardwareMapName("lf")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            ;

    //public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, .5, 1);
    public static PathConstraints pathConstraints = new PathConstraints(
            0.975,
            0.1,
            0.1,
            0.009,
            250,
            0.5,
            10,
            1
    );


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }
}
