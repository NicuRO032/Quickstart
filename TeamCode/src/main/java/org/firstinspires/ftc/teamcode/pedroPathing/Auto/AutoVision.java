package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Commands.ReadAprilTagDuringInitCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Commands.StartOuttakeFromStoredTagCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.VisionSubsystem;

@Autonomous(name = "AUTO Vision", group = "Pedro Pathing")
public class AutoVision extends CommandOpMode {

    //++private Follower follower;
    //++TelemetryData telemetryData = new TelemetryData(telemetry);
    private VisionSubsystem vision;
    private CarouselSubsystem carousel;

    private int aprilTagFromInit = -1;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private final Pose START_POSE = new Pose(24.2, 129, Math.toRadians(143));
    private final Pose SCORE_POSE = new Pose(60, 100, Math.toRadians(143));
    private final Pose PARK_POSE  = new Pose(70, 100, Math.toRadians(90));
    private PathChain scorePreloadPath;
    private PathChain parkPath;

    /**++++
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
**/
    @Override
    public void initialize() {
        //super.reset();

        //++follower = Constants.createFollower(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
        carousel = new CarouselSubsystem(hardwareMap);
        CommandScheduler.getInstance().registerSubsystem(carousel);
        CommandScheduler.getInstance().registerSubsystem(vision);



        //telemetry.addData("Status", "Initializing...");
        //telemetry.update();



        //++follower.setStartingPose(START_POSE);
        //++buildPaths();

        SequentialCommandGroup autoSequence = new SequentialCommandGroup(

                // ▶️ după START
                //new InstantCommand(() -> vision.disableProcesor()),
                new StartOuttakeFromStoredTagCommand(carousel, () -> aprilTagFromInit)

               //new FollowPathCommand(follower, scorePreloadPath, false, 0.5),


                //new FollowPathCommand(follower, parkPath, true, 0.5)
        );
        // Schedule the autonomous sequence
        schedule(autoSequence);
        //CommandScheduler.getInstance().schedule(autoSequence);

        //telemetry.addData("Status", "Initializat si gata de start!");
        //telemetry.addData("aprilTagFromInit", aprilTagFromInit);
        //++telemetry.addData("Pose X", follower.getPose().getX());
        //++telemetry.addData("Pose Y", follower.getPose().getY());
        //telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        //++follower.update();

        // 🔁 Citire activă AprilTag în INIT
        if (!isStarted() && !isStopRequested()) {
            int tag = vision.getLastTagId();
            if (tag == 21 || tag == 22 || tag == 23) {
                aprilTagFromInit = tag;

            }

            telemetry.addData("AprilTag (INIT)", aprilTagFromInit);
            telemetry.update();
            //idle(); // lasă CPU să proceseze restul
        }


        // telemetrie
        telemetry.addData("Latched AprilTag", aprilTagFromInit);
        telemetry.addData("Last Tag ID", vision.getLastTagId());
        telemetry.update();

        //telemetry.addData("AprilTag (latched)", aprilTagFromInit);
        //telemetry.update();


        //++telemetryData.addData("X", follower.getPose().getX());
        //++telemetryData.addData("Y", follower.getPose().getY());
        //++telemetryData.addData("Heading", follower.getPose().getHeading());
        //telemetry.addData("aprilTagFromInit", aprilTagFromInit);
        //telemetry.update();

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


        //packet.put("OuttakeOrder 0", carousel.getOuttakeOrder(0));
        //packet.put("OuttakeOrder 1", carousel.getOuttakeOrder(1));
        //packet.put("OuttakeOrder 2", carousel.getOuttakeOrder(2));

        packet.put("getLastTagId", vision.getLastTagId());
        packet.put("getLastBearing", vision.getLastBearing());
        packet.put("hasValidTag", vision.hasValidTag());



        dashboard.sendTelemetryPacket(packet);
    }



}
