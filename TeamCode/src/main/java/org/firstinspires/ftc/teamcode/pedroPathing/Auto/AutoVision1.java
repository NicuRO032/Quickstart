package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.pedroPathing.Commands.StartOuttakeFromStoredTagCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.VisionSubsystem;

@Autonomous(name = "AUTO Vision1", group = "Pedro Pathing")
public class AutoVision1 extends CommandOpMode {
    private VisionSubsystem vision;
    private CarouselSubsystem carousel;

    private int aprilTagFromInit = -1;
    private boolean autoStarted = false;

    @Override
    public void initialize() {

        vision = new VisionSubsystem(hardwareMap);
        carousel = new CarouselSubsystem(hardwareMap);

        CommandScheduler.getInstance().registerSubsystem(vision);
        CommandScheduler.getInstance().registerSubsystem(carousel);

        while (!isStarted() && !isStopRequested()) {

            CommandScheduler.getInstance().run();

            int tag = vision.getLastTagId();

            if (tag == 21 || tag == 22 || tag == 23) {
                aprilTagFromInit = tag;
            }

            telemetry.addLine("INIT: caut AprilTag...");
            telemetry.addData("AprilTag vazut", tag);
            telemetry.addData("AprilTag memorat", aprilTagFromInit);
            telemetry.update();
            //return;
        }


        telemetry.addLine("INIT: caut AprilTag..x.");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run(); // ⚠️ OBLIGATORIU – rulează schedulerul și periodic()

        // ▶️ O SINGURĂ DATĂ DUPĂ START
        if (!autoStarted) {
            autoStarted = true;
            telemetry.addData("START cu AprilTag", aprilTagFromInit);
            telemetry.update();
            SequentialCommandGroup autoSequence = new SequentialCommandGroup(
                    new InstantCommand(() -> vision.disableProcesor()),
                    new StartOuttakeFromStoredTagCommand(carousel, () -> aprilTagFromInit),
                    new InstantCommand(() -> vision.enableProcesor())


            );
            schedule(autoSequence);
        }
    }
}