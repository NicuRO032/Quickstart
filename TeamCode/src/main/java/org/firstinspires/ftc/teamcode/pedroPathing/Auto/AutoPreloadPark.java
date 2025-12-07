package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.OuttakeSubsystem;
@Autonomous(name = "AUTONOMIE DOAR PRELOAD", group = "Pedro Pathing")
public class AutoPreloadPark extends OpMode {

    private Follower follower;
    private OuttakeSubsystem outtake;

    private final Pose START_POSE = new Pose(24.2, 129, Math.toRadians(143));
    private final Pose SCORE_POSE = new Pose(60, 100, Math.toRadians(143));
    private final Pose PARK_POSE  = new Pose(60, 100, Math.toRadians(90));
    private Path scorePreloadPath;
    private Path parkPath;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);
        CommandScheduler.getInstance().registerSubsystem(outtake);
        follower.setStartingPose(START_POSE);
        scorePreloadPath = new Path(new BezierLine(START_POSE, SCORE_POSE));
        scorePreloadPath.setLinearHeadingInterpolation(START_POSE.getHeading(), SCORE_POSE.getHeading());

        parkPath = new Path(new BezierLine(SCORE_POSE, PARK_POSE));
        parkPath.setLinearHeadingInterpolation(SCORE_POSE.getHeading(), PARK_POSE.getHeading());

        SequentialCommandGroup autoSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, scorePreloadPath, true, 0.5),
                new InstantCommand(() -> {
                    outtake.setShooterPower(0.5);
                    outtake.setServoPosition(0.85);
                }),
                new WaitCommand(2000),
                new InstantCommand(() -> {
                    outtake.stop();
                }),

                new FollowPathCommand(follower, parkPath, true, 0.5)
        );
        CommandScheduler.getInstance().schedule(autoSequence);
    }

    @Override
    public void init_loop() {

        telemetry.addData("Status", "Initializat si gata de start!");
        telemetry.addData("Pose X", follower.getPose().getX());
        telemetry.addData("Pose Y", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

        follower.update();
        CommandScheduler.getInstance().run();

        telemetry.addData("RobotX", follower.getPose().getX());
        telemetry.addData("RobotY", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
    }
}
