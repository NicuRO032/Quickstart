package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.6)
            .forwardZeroPowerAcceleration(132.13018574414062)
            .lateralZeroPowerAcceleration(66.6640833142628)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.00001, 0, 0.0005, 0.0007))
            .headingPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.0015, 0.005))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0004,0,0.01,0.6,0.003))
            .centripetalScaling(0.0007);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);
            //.xVelocity(134.97044975852518)
            //.yVelocity(49.07182148731963);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(.0291982274277294)
            .strafeTicksToInches(.0608545201363458)
            .turnTicksToInches(.0027987026270567)
            .leftPodY(2.95)
            .rightPodY(-2.95)
            .strafePodX(-3.75)
            .leftEncoder_HardwareMapName("lf")
            .rightEncoder_HardwareMapName("rr")
            .strafeEncoder_HardwareMapName("rf")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                //.pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }
}




