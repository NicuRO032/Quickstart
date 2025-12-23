package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.VisionSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.function.Supplier;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;


//@Configurable

@TeleOp(name="TeleOpCarousel")
@Config
public class TeleOpCarousel extends OpMode {

    private GamepadEx driver1;
    private GamepadEx driver2;

    public static boolean simulateDpadLeft = false;
    public static boolean simulateDpadRight = false;


    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;


    private CarouselSubsystem carousel;
    private VisionSubsystem vision;

    private boolean isProcessingVideo = true;
    private int idAprilTag = 0;
    private double cameraAngle = 0.0d;

    FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void init() {
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        // Initialize subsystems
        carousel = new CarouselSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
        //vision = new VisionSubsystem(hardwareMap);

        // Register subsystems in the CommandScheduler
        CommandScheduler.getInstance().registerSubsystem(carousel);
        CommandScheduler.getInstance().registerSubsystem(vision);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

        //Call this once per loop
        CommandScheduler.getInstance().run();

        driver1.readButtons();
        driver2.readButtons();

        if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)
                || simulateDpadLeft) {
            carousel.enableAuto(false);
            carousel.manualStepLeft();
        }

        if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)
                || simulateDpadRight) {
            carousel.enableAuto(false);
            carousel.manualStepRight();
        }

        if (driver1.wasJustPressed(GamepadKeys.Button.X))
            carousel.startOuttake(CarouselSubsystem.OuttakePattern.PGG);

        if (driver1.wasJustPressed(GamepadKeys.Button.Y))
            carousel.startOuttake(CarouselSubsystem.OuttakePattern.GPG);

        if (driver1.wasJustPressed(GamepadKeys.Button.B))
            carousel.startOuttake(CarouselSubsystem.OuttakePattern.GGP);

        // Save CPU resources; can resume streaming when needed.
        if (gamepad1.dpad_down && isProcessingVideo) {
            vision.disableProcesor();
            isProcessingVideo = false;
        }
        if (gamepad1.dpad_up && !isProcessingVideo) {
            vision.enableProcesor();
            //visionPortal.setProcessorEnabled(aprilTag, true);
            //visionPortal.resumeStreaming();
            isProcessingVideo = true;
        }
/**
        idAprilTag = 0;
        cameraAngle = 0.0d;
        if (isProcessingVideo){
            AprilTagDetection tag = vision.getBestDetection();
            if (tag != null && tag.metadata != null) {
                idAprilTag = tag.id;
                if (idAprilTag == 20 || idAprilTag == 24) {
                    cameraAngle = tag.ftcPose.bearing;
                }
            }


        }
**/
        // 6️⃣ Trimite datele la Dashboard (pentru grafic)
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Index", carousel.getIndex());
        packet.put("logicalIndex", carousel.getLogicalIndex());
        packet.put("CarouselTarget Position", carousel.getTargetPosition());
        packet.put("Actual Position", carousel.getCurrentPosition());
        packet.put("Occupied 0", carousel.getOccupied(0));
        packet.put("Occupied 1", carousel.getOccupied(1));
        packet.put("Occupied 2", carousel.getOccupied(2));
        packet.put("Color 0", carousel.getBallColor(0));
        packet.put("Color 1", carousel.getBallColor(1));
        packet.put("Color 2", carousel.getBallColor(2));
        packet.put("Hue1", carousel.getHue1());
        packet.put("Hue2", carousel.getHue2());
        packet.put("HueMax", carousel.getHueMax());
        packet.put("CurrentColor", carousel.getBallColor());
        packet.put("IntakeState", carousel.getIntakeState());
        packet.put("OuttakeState", carousel.getOuttakeState());
        packet.put("Distance", carousel.getDistance());
        packet.put("isProcessingVideo", isProcessingVideo);
        packet.put("Tag ID", idAprilTag);
        packet.put("Unghi (Bearing):", cameraAngle);

        packet.put("getLastTagId", vision.getLastTagId());
        packet.put("getLastBearing", vision.getLastBearing());
        packet.put("hasValidTag", vision.hasValidTag());



        dashboard.sendTelemetryPacket(packet);
    }
}