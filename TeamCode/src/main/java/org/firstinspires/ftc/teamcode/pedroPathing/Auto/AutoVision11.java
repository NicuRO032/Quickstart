package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Commands.PrepareOuttakeFromTagCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Commands.ShootAllBallsCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Commands.StartOuttakeFromStoredTagCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem1;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.VisionSubsystem;

@Autonomous(name = "AUTO Vision11", group = "Pedro Pathing")
public class AutoVision11 extends CommandOpMode {
    private VisionSubsystem vision;
    private CarouselSubsystem1 carousel;

    private int aprilTagFromInit = -1;
    private double correctionAngle = 0.0d;
    private boolean autoStarted = false;

    @Override
    public void initialize() {

        vision = new VisionSubsystem(hardwareMap);
        carousel = new CarouselSubsystem1(hardwareMap);

        CommandScheduler.getInstance().registerSubsystem(vision);
        CommandScheduler.getInstance().registerSubsystem(carousel);

        // Setare bile preîncărcate chiar înainte de start
        carousel.forcePreload(CarouselSubsystem1.BallColor.PURPLE, CarouselSubsystem1.BallColor.GREEN, CarouselSubsystem1.BallColor.GREEN);

        while (!isStarted() && !isStopRequested()) {

            CommandScheduler.getInstance().run();

            int tag = vision.getLastTagId();

            if (tag == 21 || tag == 22 || tag == 23) {
                aprilTagFromInit = tag;
            }

            telemetry.addLine("INIT: caut AprilTag...");
            telemetry.addData("AprilTag vazut", tag);
            telemetry.addData("BearingAngle", vision.getLastBearing());
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

        telemetry.addData("Correction Angle", correctionAngle);
        telemetry.update();

        // ▶️ O SINGURĂ DATĂ DUPĂ START
        if (!autoStarted) {
            autoStarted = true;
            telemetry.addData("START cu AprilTag", aprilTagFromInit);
            telemetry.update();
            SequentialCommandGroup autoSequence = new SequentialCommandGroup(
                    new PrepareOuttakeFromTagCommand(carousel, aprilTagFromInit),
                    new ShootAllBallsCommand(carousel)
            );
            schedule(autoSequence);
        }
    }
}