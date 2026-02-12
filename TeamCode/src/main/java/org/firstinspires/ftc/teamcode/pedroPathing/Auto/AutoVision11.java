package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Commands.AutoAimTurretCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Commands.DetectAprilTagCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Commands.PrepareOuttakeFromTagCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Commands.ShootAllBallsCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem1;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.IntakeSubsystem1;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.VisionSubsystem;

@Autonomous(name = "AUTO.big.BLUE", group = "Pedro Pathing")
public class AutoVision11 extends CommandOpMode {
    private Follower follower;
    private CarouselSubsystem1 carousel;
    private TurretSubsystem turret;
    private VisionSubsystem vision;
    private IntakeSubsystem1 intake;
    private FtcDashboard dashboard;
    private int aprilTagFromInit = -1;
    private double correctionAngle = 0.0d;
    private boolean autoStarted = false;
    double coarseTurretShootingAngle = 0.0;


    public static PathConstraints FAST_CONSTRAINTS = new PathConstraints(
            0.1,  // 90% din viteza maximă
            100,  // Accelerație mare
            1.3,  // Viteză angulară mare
            1.0);

    public static PathConstraints SLOW_CONSTRAINTS = new PathConstraints(
            0.1,  // 40% din viteza maximă
            10,   // Accelerație mai mică, pentru mișcări line
            0.1,  // Viteză angulară mai mică
            0.1);

    // Definește toate punctele cheie ale autonomiei
    private final Pose START_POSE = new Pose(14.5, 120, Math.toRadians(90));
    private final Pose SCORE_POSE = new Pose(52, 90.5, Math.toRadians(133));
    private final Pose PARK_POSE  = new Pose(52.5, 75.5, Math.toRadians(135));
    private final Pose GRAB1_START_POSE  = new Pose(18, 84, Math.toRadians(180));
    private final Pose GRAB1_END_POSE  = new Pose(18, 84, Math.toRadians(180));
    private final Pose ControlPoint1 = new Pose(59,81);
    private final Pose ControlPoint = new Pose(64, 56);
    private final Pose GRAB2_START_POSE  = new Pose(47, 66, Math.toRadians(180));
    private final Pose GRAB2_END_POSE  = new Pose(11, 60, Math.toRadians(180));
    private final Pose GRAB3_START_POSE  = new Pose(60, 100, Math.toRadians(143));
    private final Pose GRAB3_END_POSE  = new Pose(60, 100, Math.toRadians(143));

    private PathChain scorePreloadPath;
    private PathChain parkPath;
    private PathChain grab1Path;
    private PathChain grab1APath;
    private PathChain Controlpoint;
    private PathChain grab2Path;
    private PathChain grab2APath;
    private PathChain grab3Path;
    private PathChain score1Path;
    private PathChain score2Path;
    private PathChain score3Path;

    public void buildPaths() {
        // 1. De la START la SCOR (Preload)
        scorePreloadPath = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, SCORE_POSE))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), SCORE_POSE.getHeading())
                .build();

        // 2. Traiectoria de colectare 1 (de la SCOR la zona de colectare)
        grab1Path = follower.pathBuilder()
                .addPath(new BezierCurve(SCORE_POSE, ControlPoint1, GRAB1_END_POSE)) // Pleacă de la SCORE_POSE
                .setLinearHeadingInterpolation(SCORE_POSE.getHeading(), GRAB1_END_POSE.getHeading())
                .addParametricCallback(0.0, () -> follower.setMaxPower(1))
                .addParametricCallback(0.25, () -> follower.setMaxPower(0.35))
                .addPoseCallback(GRAB1_END_POSE, () -> intake.setPower(-1), 7)
                //.setConstraints(SLOW_CONSTRAINTS)
                .build();

        /*grab1APath = follower.pCoathBuilder()
                .addPath(new BezierLine(GRAB1_START_POSE, GRAB1_END_POSE))
                .setLinearHeadingInterpolation(GRAB1_START_POSE.getHeading(), GRAB1_END_POSE.getHeading())
                .build();*/

        // 3. Traiectoria de scor 1 (de la COLECTARE înapoi la SCOR)
        score1Path = follower.pathBuilder()
                .addPath(new BezierLine(GRAB1_END_POSE, SCORE_POSE)) // Pleacă de unde a terminat colectarea
                .setLinearHeadingInterpolation(GRAB1_END_POSE.getHeading(), SCORE_POSE.getHeading())
                .addParametricCallback(0.0, () -> follower.setMaxPower(1.0))

                .build();

        // 4. Traiectoria de colectare 2 (de la SCOR la a doua zonă de colectare)
        grab2Path = follower.pathBuilder()
                .addPath(new BezierCurve(SCORE_POSE, ControlPoint, GRAB2_END_POSE)) // Pleacă de la SCORE_POSE
                .setLinearHeadingInterpolation(SCORE_POSE.getHeading(), GRAB2_END_POSE.getHeading())
                .addParametricCallback(0.0, () -> follower.setMaxPower(1.0))
                .addParametricCallback(0.55, () -> follower.setMaxPower(0.32))
                .addPoseCallback(GRAB2_END_POSE, () -> intake.setPower(-1), 7) // Corectat din GRAB1_START_POSE
                // .setConstraints(SLOW_CONSTRAINTS)
                .build();

       /* grab2APath = follower.pathBuilder()
                .addPath(new BezierLine(GRAB2_START_POSE, GRAB2_END_POSE))
                .setLinearHeadingInterpolation(GRAB2_START_POSE.getHeading(), GRAB2_END_POSE.getHeading())
                .build(); */
        // 5. Traiectoria de scor 2 (de la COLECTARE 2 înapoi la SCOR)
        score2Path = follower.pathBuilder()
                .addPath(new BezierLine(GRAB2_END_POSE, SCORE_POSE))
                .addParametricCallback(0.0, () -> follower.setMaxPower(1.0))
                .setLinearHeadingInterpolation(GRAB2_END_POSE.getHeading(), SCORE_POSE.getHeading())
                .build();

        // --- Restul traiectoriilor urmează același model ---

       /* grab3Path = follower.pathBuilder()
                .addPath(new BezierLine(SCORE_POSE, GRAB3_START_POSE))
                .setLinearHeadingInterpolation(SCORE_POSE.getHeading(), GRAB3_START_POSE.getHeading())
                .setConstraints(SLOW_CONSTRAINTS)
                .addPath(new BezierLine(GRAB3_START_POSE, GRAB3_END_POSE))
                .setLinearHeadingInterpolation(GRAB3_START_POSE.getHeading(), GRAB3_END_POSE.getHeading())
                .addPoseCallback(GRAB3_START_POSE, () -> intake.setPower(0.8), 0.5) // Corectat
                .build();

        score3Path = follower.pathBuilder()
                .addPath(new BezierLine(GRAB3_END_POSE, SCORE_POSE))
                .setLinearHeadingInterpolation(GRAB3_END_POSE.getHeading(), SCORE_POSE.getHeading())
                .build();*/

        // Traiectoria de parcare
        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(SCORE_POSE, PARK_POSE))
                .setLinearHeadingInterpolation(SCORE_POSE.getHeading(), PARK_POSE.getHeading())
                .build();
    }


    @Override
    public void initialize() {
        dashboard = FtcDashboard.getInstance();
        follower = Constants.createFollower(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
        carousel = new CarouselSubsystem1(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        intake = new IntakeSubsystem1(hardwareMap);


        // Apelează buildPaths() AICI, după ce follower-ul a fost inițializat
        follower.setStartingPose(START_POSE);
        buildPaths();

        carousel.resetForStart();

        CommandScheduler.getInstance().registerSubsystem(vision);
        CommandScheduler.getInstance().registerSubsystem(carousel);
        CommandScheduler.getInstance().registerSubsystem(turret);
        CommandScheduler.getInstance().registerSubsystem(intake);

        // Setare bile preîncărcate chiar înainte de start
        carousel.forcePreload(CarouselSubsystem1.BallColor.GREEN, CarouselSubsystem1.BallColor.PURPLE, CarouselSubsystem1.BallColor.PURPLE);
        carousel.setShooterForAutoRPM(3800);
        turret.setTargetAngle(-55);
        turret.setShooterAngle(0.1);
        vision.enableProcesor();

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
        }

        telemetry.addLine("INIT: gata de start.");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run(); //OBLIGATORIU – rulează schedulerul și periodic()
        follower.update();

        telemetry.addData("Correction Angle", correctionAngle);
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("IntakeState", carousel.getIntakeState());
        packet.put("OuttakeState", carousel.getOuttakeState());
        packet.put("Logical Index", carousel.getLogicalIndex());
        packet.put("Carousel Logical Index", carousel.getLogicalIndex());
        packet.put("Carousel Target Feedback (mV)", carousel.getTargetFeedbackMv());
        packet.put("Carousel Current Feedback (mV)", carousel.getCurrentFeedbackMv());
        packet.put("Carousel Feedback Error (mV)", carousel.getFeedbackError());
        packet.put("Carousel At Target", carousel.atTarget());

        packet.put("Order", carousel.getOuttakeOrderString());
        packet.put("Ptr", carousel.getOuttakePtr());

        packet.put("Slots Occupied", String.format("[%b, %b, %b]",
                carousel.getOccupied(0), carousel.getOccupied(1), carousel.getOccupied(2)));
        packet.put("Slots Colors", carousel.getSlotsColorString());

        dashboard.sendTelemetryPacket(packet);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();


        // O SINGURĂ DATĂ DUPĂ START
        if (!autoStarted) {
            autoStarted = true;
            follower.setStartingPose(START_POSE);
            telemetry.addData("START cu AprilTag", aprilTagFromInit);
            telemetry.update();

            SequentialCommandGroup autoSequence = new SequentialCommandGroup(
                    //--- CICLUL 1: SCOR PRELOAD ---
                    new ParallelCommandGroup(
                            new InstantCommand(() -> follower.setMaxPower(1)),
                            new InstantCommand(() -> turret.setTargetAngle(0))
                    ),
                    /*
                    new InstantCommand(() -> turret.setTargetAngle(-55)),
                    new ParallelCommandGroup(
                            new FollowPathCommand(follower, scorePreloadPath, false),
                            new DetectAprilTagCommand(vision, (tagId) -> this.aprilTagFromInit = tagId, 1500)
                    ),
                    */
                    new FollowPathCommand(follower, scorePreloadPath, false),
                    //new WaitCommand(200),
                    // Pregătește caruselul pentru outtake și pornește shooter-ul
                    new PrepareOuttakeFromTagCommand(carousel, () -> this.aprilTagFromInit),
                    new InstantCommand(() -> turret.setTargetAngle(0)),

                    new ParallelRaceGroup(
                            new WaitCommand(1500),
                            new AutoAimTurretCommand(turret, vision,20)
                    ),
                    // Acum, comandă tragerea
                    new ShootAllBallsCommand(carousel),
                    //new WaitCommand(1000),
                    new InstantCommand(() -> intake.setPower(-0.8)),

                    //--- CICLUL 2: PRIMA COLECTARE ȘI SCOR ---
                    new ParallelRaceGroup(
                            new WaitUntilCommand(carousel::allSlotsOccupied),
                            new FollowPathCommand(follower, grab1Path, false),
                            new WaitCommand(5000)
                    ),
                    new InstantCommand(() -> intake.setPower(0)),
                    new InstantCommand(() -> follower.setMaxPower(0.8)),
                    new InstantCommand(() -> intake.setPower(0.4)),
                    new ParallelCommandGroup(
                            new PrepareOuttakeFromTagCommand(carousel, () -> this.aprilTagFromInit),
                            new FollowPathCommand(follower, score1Path, false)
                    ),
                    new InstantCommand(() -> intake.setPower(0)),
                    new ParallelRaceGroup(
                            new WaitCommand(1500),
                            new AutoAimTurretCommand(turret, vision,20)
                    ),
                    new ShootAllBallsCommand(carousel),

                    //--- CICLUL 3: A doua colectare si scor ---
                    new InstantCommand(() -> follower.setMaxPower(0.8)),

                    new InstantCommand(() -> intake.setPower(-0.8)),
                    new ParallelRaceGroup(
                            new WaitUntilCommand(carousel::allSlotsOccupied),
                            new FollowPathCommand(follower, grab2Path, false),
                            new WaitCommand(5000)
                    ),
                    new InstantCommand(() -> intake.setPower(0)),
                    new InstantCommand(() -> follower.setMaxPower(0.5)),
                    new ParallelCommandGroup(
                            new PrepareOuttakeFromTagCommand(carousel, () -> this.aprilTagFromInit),
                            new FollowPathCommand(follower, score2Path, false),
                            new InstantCommand(() -> intake.setPower(1))
                    ),
                    new InstantCommand(() -> intake.setPower(0)),
                    new ParallelRaceGroup(
                            new WaitCommand(1500),
                            new AutoAimTurretCommand(turret, vision,20)
                    ),
                    new ShootAllBallsCommand(carousel),
                    new InstantCommand(() -> follower.setMaxPower(1)),
                    new FollowPathCommand(follower, parkPath, false)
                    /*  // A doua colectare (comentat)
                    new ParallelRaceGroup(
                            new FollowPathCommand(follower, grab2Path, false, 0.9),
                            new WaitUntilCommand(carousel::allSlotsOccupied),
                            new WaitCommand(5000)
                    ),
                    new InstantCommand(() -> intake.setPower(0)),
                    new ParallelCommandGroup(
                            new PrepareOuttakeFromTagCommand(carousel, aprilTagFromInit),
                            new FollowPathCommand(follower, score2Path, true, 0.5)
                    ),
                    new WaitCommand(1000),
                    new ShootAllBallsCommand(carousel)
                    */
            );
            schedule(autoSequence);
        }
    }
}
