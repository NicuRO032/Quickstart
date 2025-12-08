package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Commands.RunOuttakeSequenceCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.OuttakeSubsystem;

@Autonomous(name = "AUTONOMIE DOAR PRELOAD OK", group = "Pedro Pathing")
public class AutoPreloadParkOK extends CommandOpMode {

    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private OuttakeSubsystem outtake;
    private IntakeSubsystem intake;

    private final Pose START_POSE = new Pose(24.2, 129, Math.toRadians(143));
    private final Pose SCORE_POSE = new Pose(60, 100, Math.toRadians(143));
    private final Pose PARK_POSE  = new Pose(70, 100, Math.toRadians(90));
    private PathChain scorePreloadPath;
    private PathChain parkPath;

    public void buildPaths() {
        scorePreloadPath = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, SCORE_POSE))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), SCORE_POSE.getHeading())
                .build();

        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(SCORE_POSE, PARK_POSE))
                .setLinearHeadingInterpolation(SCORE_POSE.getHeading(), PARK_POSE.getHeading())
                .build();
    }

    @Override
    public void initialize() {
        //super.reset();

        follower = Constants.createFollower(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        //CommandScheduler.getInstance().registerSubsystem(outtake);
        //CommandScheduler.getInstance().registerSubsystem(intake);

        follower.setStartingPose(START_POSE);
        buildPaths();

        SequentialCommandGroup autoSequence = new SequentialCommandGroup(

                new FollowPathCommand(follower, scorePreloadPath, false, 0.5),
                new RunOuttakeSequenceCommand(intake, outtake),

                new FollowPathCommand(follower, parkPath, true, 0.5)
        );
        // Schedule the autonomous sequence
        schedule(autoSequence);
        //CommandScheduler.getInstance().schedule(autoSequence);

        telemetry.addData("Status", "Initializat si gata de start!");
        telemetry.addData("Pose X", follower.getPose().getX());
        telemetry.addData("Pose Y", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        follower.update();


        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }



}
