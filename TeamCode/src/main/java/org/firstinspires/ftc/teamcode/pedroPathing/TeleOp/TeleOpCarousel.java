package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Commands.AutoStoreCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Commands.RunOuttakeSequenceCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.OuttakeSubsystem;

import java.util.function.Supplier;


@Configurable
@TeleOp(name="TeleOpCarousel")
public class TeleOpCarousel extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private CarouselSubsystem carousel;


    public boolean sequenceRunning = false;
    FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void init() {


        // Initialize subsystems

        carousel = new CarouselSubsystem(hardwareMap);


        // Register subsystems in the CommandScheduler
        CommandScheduler.getInstance().registerSubsystem(carousel);


    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        //Call this once per loop
        //telemetryM.update();
        CommandScheduler.getInstance().run();

        CommandScheduler.getInstance().schedule(
                new AutoStoreCommand(carousel));


        // controale manuale opționale (debug)
        if (gamepad1.dpadLeftWasPressed()) {
            carousel.stepLeft();
        }
        if (gamepad1.dpadRightWasPressed()) {
            carousel.stepRight();
        }

        // 6️⃣ Trimite datele la Dashboard (pentru grafic)
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Index", carousel.getIndex());
        packet.put("logicalIndex", carousel.getLogicalIndex());
        packet.put("CarouselTarget Position", carousel.getTargetPosition());
        packet.put("Actual Position", carousel.getCurrentPosition());
        //packet.put("Actual Power", carousel.motor.getPower());
        dashboard.sendTelemetryPacket(packet);


    }
}