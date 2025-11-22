package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Example Auto Clean", group = "Pedro Pathing")
public class ExampleAutoSolversLib extends OpMode {

    private Follower follower;

    private final Pose startPose = new Pose(36, 135.5, Math.toRadians(180));
    private final Pose scorePose = new Pose(59, 84, Math.toRadians(143));
    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0));

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // Construim path-urile
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        follower.setStartingPose(startPose);

        // Construim secvența de comenzi cu blocking FollowPathCommand
        SequentialCommandGroup autoSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, scorePreload, true, 1.0),   // blocking
                new FollowPathCommand(follower, grabPickup1, true, 1.0),
                new FollowPathCommand(follower, scorePickup1, true, 1.0),
                new FollowPathCommand(follower, grabPickup2, true, 1.0),
                new FollowPathCommand(follower, scorePickup2, true, 1.0),
                new FollowPathCommand(follower, grabPickup3, true, 1.0),
                new FollowPathCommand(follower, scorePickup3, true, 1.0)
        );

        // Programăm secvența în CommandScheduler
        CommandScheduler.getInstance().schedule(autoSequence);
    }

    @Override
    public void loop() {
        // Actualizăm Pedro Pathing și rulăm schedulerul SolversLib
        follower.update();
        CommandScheduler.getInstance().run();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
