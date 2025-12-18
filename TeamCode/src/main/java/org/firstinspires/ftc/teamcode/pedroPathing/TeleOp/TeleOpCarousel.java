package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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


//@Configurable

@TeleOp(name="TeleOpCarousel")
@Config
public class TeleOpCarousel extends OpMode {

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
    // pentru detectare front crescător
    private boolean lastBallState = false;


    //public boolean sequenceRunning = false;
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



        /* ================= AUTO STORE ================= */

        boolean ballDetected = carousel.entrySlotHasBall(); // deja delay-uit în subsystem

        // pornește AutoStore o singură dată per bilă
        if (ballDetected && !lastBallState && !carousel.allSlotsOccupied()) {
            CommandScheduler.getInstance().schedule(
                    new AutoStoreCommand(carousel)
            );
        }

        lastBallState = ballDetected;

        /* ================= MANUAL OVERRIDE ================= */

        boolean leftPressed = gamepad1.dpadLeftWasPressed() || simulateDpadLeft;
        boolean rightPressed = gamepad1.dpadRightWasPressed() || simulateDpadRight;

        if (leftPressed) {
            CommandScheduler.getInstance().cancelAll();
            carousel.stepLeft();
            simulateDpadLeft = false; // resetăm după simulare
        }

        if (rightPressed) {
            CommandScheduler.getInstance().cancelAll();
            carousel.stepRight();
            simulateDpadRight = false; // resetăm după simulare
        }


        // 6️⃣ Trimite datele la Dashboard (pentru grafic)
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Index", carousel.getIndex());
        packet.put("logicalIndex", carousel.getLogicalIndex());
        packet.put("CarouselTarget Position", carousel.getTargetPosition());
        packet.put("Actual Position", carousel.getCurrentPosition());
        packet.put("Occupied 0", carousel.getOccupied(0));
        packet.put("Occupied 1", carousel.getOccupied(1));
        packet.put("Occupied 2", carousel.getOccupied(2));
        //packet.put("Actual Power", carousel.motor.getPower());
        dashboard.sendTelemetryPacket(packet);


    }
}