package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Commands.RunIntakeCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Commands.RunOuttakeCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.OuttakeSubsystem;

@Autonomous(name = "Example Auto Parallel Intake-Outtake", group = "Pedro Pathing")
public class ExampleAutoIntakeOuttakeParallel extends OpMode {

    private Follower follower;
    private IntakeSubsystem intake;
    private OuttakeSubsystem outtake;

    private final Pose startPose = new Pose(28.5, 128, Math.toRadians(180));
    private final Pose pickupPose = new Pose(50, 130, Math.toRadians(0));
    private final Pose scorePose = new Pose(60, 90, Math.toRadians(135));

    private Path toPickup;
    private PathChain toScore;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // Inițializăm subsistemele
        intake = new IntakeSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);

        // Înregistrăm subsistemele în CommandScheduler
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.registerSubsystem(intake);
        scheduler.registerSubsystem(outtake);

        // Construim path-urile
        toPickup = new Path(new BezierLine(startPose, pickupPose));
        toPickup.setLinearHeadingInterpolation(startPose.getHeading(), pickupPose.getHeading());

        toScore = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, scorePose))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), scorePose.getHeading())
                .build();

        follower.setStartingPose(startPose);

        // 🚀 Autonomie combinată (paralel)
        SequentialCommandGroup autoSequence = new SequentialCommandGroup(
                // 1️⃣ Mergem către pickup și rulăm intake-ul simultan
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, toPickup, true, 1.0),
                        new RunIntakeCommand(intake, 1.0)
                ),

                // 2️⃣ Mergem către zona de scoring și rulăm outtake-ul simultan
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, toScore, true, 1.0),
                        new RunOuttakeCommand(outtake, 0.8, 1.0)
                )
        );

        // Programăm secvența în CommandScheduler
        scheduler.schedule(autoSequence);
    }

    @Override
    public void loop() {
        follower.update();
        CommandScheduler.getInstance().run();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
