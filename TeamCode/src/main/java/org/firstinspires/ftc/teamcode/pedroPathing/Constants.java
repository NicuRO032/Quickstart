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
            .mass(13.8)
            .forwardZeroPowerAcceleration(-39.68006229544917)
            .lateralZeroPowerAcceleration(-75.90603183464327)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.005, 0.015))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.05, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02,0.0,0.0001,0.6,0.001))
            .centripetalScaling(0.00057)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(58.9961357680407)
            .yVelocity(49.07686353560646)
            ;

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .leftPodY(3.14)
            .rightPodY(-3.14)
            .strafePodX(-4.52)
            .forwardTicksToInches(.001948677514836943)
            .strafeTicksToInches(.0019876010767738263)
            .turnTicksToInches(.0018844454715644498)
            .leftEncoder_HardwareMapName("lr")
            .rightEncoder_HardwareMapName("rf")
            .strafeEncoder_HardwareMapName("rr")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            ;


    public static PathConstraints pathConstraints = new PathConstraints(1.5, 100, 1, 1);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }
}