package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Commands.PrepareOuttakeFromTagCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Commands.ShootAllBallsCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem1;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.IntakeSubsystem1;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.TurretSubsystem;

@Autonomous(name = "AUTO.BIG.BLUE", group = "Pedro Pathing")
public class AutoVision11 extends CommandOpMode {
    private Follower follower;
    private CarouselSubsystem1 carousel;
    private TurretSubsystem turret;
//    private VisionSubsystem vision;
    private IntakeSubsystem1 intake;
    private FtcDashboard dashboard;
    private int aprilTagFromInit = -1;
  //  private double correctionAngle = 0.0d;
    private boolean autoStarted = false;

    // Definește toate punctele cheie ale autonomiei
    private final Pose START_POSE = new Pose(21, 124, Math.toRadians(143));
    private final Pose SCORE_POSE = new Pose(52, 90.5, Math.toRadians(133));
    private final Pose PARK_POSE  = new Pose(52.5, 85, Math.toRadians(135));
    private final Pose GRAB1_END_POSE = new Pose(12, 66, Math.toRadians(180));
    private final Pose GRAB2_END_POSE = new Pose(15 , 87, Math.toRadians(180));
    private final Pose GRAB3_END_POSE = new Pose(2 , 38, Math.toRadians(180));
    private final Pose ControlPoint1 = new Pose(50,39);
    private final Pose ControlPoint2 = new Pose(31, 62);
    private final Pose ControlPoint3 = new Pose(81, 37);
    private final Pose ControlPoint4 = new Pose(65, 81);


    private PathChain scorePreloadPath;
    private PathChain parkPath;
    private PathChain grab1Path;
    private PathChain grab2Path;
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
                .addPath(new BezierCurve(SCORE_POSE, ControlPoint1, GRAB1_END_POSE)) // set 2 artefacte
                .setLinearHeadingInterpolation(SCORE_POSE.getHeading(), GRAB1_END_POSE.getHeading())
                .addParametricCallback(0.0, () -> follower.setMaxPower(1))
                .addParametricCallback(0.55, () -> follower.setMaxPower(0.45))
                .addParametricCallback(0.8, () -> follower.setMaxPower(1))
                .build();

        // 3. Traiectoria de scor 1 (de la COLECTARE înapoi la SCOR)
        score1Path = follower.pathBuilder()
                .addPath(new BezierCurve(GRAB1_END_POSE, ControlPoint2, SCORE_POSE)) // Pleacă de unde a terminat colectarea
                .setLinearHeadingInterpolation(GRAB1_END_POSE.getHeading(), SCORE_POSE.getHeading())
                .addParametricCallback(0.90, () -> follower.setMaxPower(0.9))
                .build();

        // 4. Traiectoria de colectare 2 (de la SCOR la a doua zonă de colectare)
        grab2Path = follower.pathBuilder()
                .addPath(new BezierCurve(SCORE_POSE, ControlPoint4, GRAB2_END_POSE)) //  primul set
                .setLinearHeadingInterpolation(SCORE_POSE.getHeading(), GRAB2_END_POSE.getHeading())
                .addParametricCallback(0.0, () -> follower.setMaxPower(1))
                .addParametricCallback(0.4, () -> follower.setMaxPower(0.4))
                .build();

        //7. Traiectoria de la Colectare 3 la score 3
        score2Path = follower.pathBuilder()
                .addPath(new BezierLine(GRAB2_END_POSE, SCORE_POSE))
                .addParametricCallback(0.0, () -> follower.setMaxPower(1.0))
                .addParametricCallback(0.85, () -> follower.setMaxPower(0.9))
                .setLinearHeadingInterpolation(GRAB2_END_POSE.getHeading(), SCORE_POSE.getHeading())
                .build();

        // 6. Traiectoria de la score 2 la COLECTARE 3
        grab3Path = follower.pathBuilder()
                .addPath(new BezierCurve(SCORE_POSE, ControlPoint3, GRAB3_END_POSE)) // Pleacă de la SCORE_POSE
                .setLinearHeadingInterpolation(SCORE_POSE.getHeading(), GRAB3_END_POSE.getHeading())
                .addParametricCallback(0.0, () -> follower.setMaxPower(1))
                .addParametricCallback(0.8, () -> follower.setMaxPower(0.4))
                .build();

        //7. Traiectoria de la Colectare 3 la score 3
        score3Path = follower.pathBuilder()
                .addPath(new BezierLine(GRAB3_END_POSE, SCORE_POSE))
                .addParametricCallback(0.0, () -> follower.setMaxPower(1.0))
                .setLinearHeadingInterpolation(GRAB3_END_POSE.getHeading(), SCORE_POSE.getHeading())
                .build();

        // Traiectoria de parcare
        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(SCORE_POSE, PARK_POSE))
                .setLinearHeadingInterpolation(SCORE_POSE.getHeading(), PARK_POSE.getHeading())
                .addParametricCallback(0.0, () -> follower.setMaxPower(1))
                .build();
    }


    @Override
    public void initialize() {
        dashboard = FtcDashboard.getInstance();
        follower = Constants.createFollower(hardwareMap);
        //vision = new VisionSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        intake = new IntakeSubsystem1(hardwareMap);
        carousel = new CarouselSubsystem1(hardwareMap, intake);
        carousel.isTeleOp = false;

        // Apelează buildPaths() AICI, după ce follower-ul a fost inițializat
        follower.setStartingPose(START_POSE);
        buildPaths();

        carousel.resetForStart();

       // CommandScheduler.getInstance().registerSubsystem(vision);
        CommandScheduler.getInstance().registerSubsystem(carousel);
        CommandScheduler.getInstance().registerSubsystem(turret);
        CommandScheduler.getInstance().registerSubsystem(intake);

        // Setare bile preîncărcate chiar înainte de start
        //carousel.forcePreload(CarouselSubsystem1.BallColor.GREEN, CarouselSubsystem1.BallColor.PURPLE, CarouselSubsystem1.BallColor.PURPLE);
        //carousel.setShooterForAutoRPM(3650);
      //  turret.setTargetAngle(-55);
        turret.setShooterAngle(0.15);
        //vision.enableProcesor();



        telemetry.addLine("INIT: gata de start.");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run(); //OBLIGATORIU – rulează schedulerul și periodic()
        follower.update();

//        telemetry.addData("Correction Angle", correctionAngle);
//        telemetry.update();
//
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("01 IntakeState", carousel.getIntakeState());
//        packet.put("02 OuttakeState", carousel.getOuttakeState());
//        packet.put("02 SlowShootState", carousel.getSlowShootState());
//        packet.put("03 IsShooterReady", carousel.isShooterReady());
//        packet.put("04 IsReadyToShoot", carousel.getIsReadyToShoot());
//        packet.put("Logical Index", carousel.getLogicalIndex());
//        packet.put("Carousel Logical Index", carousel.getLogicalIndex());
//        packet.put("Carousel Target Feedback (mV)", carousel.getTargetFeedbackMv());
//        packet.put("Carousel Current Feedback (mV)", carousel.getCurrentFeedbackMv());
//        packet.put("Carousel Feedback Error (mV)", carousel.getFeedbackError());
//        packet.put("Carousel At Target", carousel.atTarget());
//
//
//
//        packet.put("Slots Occupied", String.format("[%b, %b, %b]",
//                carousel.getOccupied(0), carousel.getOccupied(1), carousel.getOccupied(2)));
//        packet.put("Slots Colors", carousel.getSlotsColorString());
//        packet.put("AprilTag Vazut", aprilTagFromInit);
//
//        dashboard.sendTelemetryPacket(packet);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.addData("AprilTag Vazut", aprilTagFromInit);
//        telemetry.update();
//

        // O SINGURĂ DATĂ DUPĂ START
        if (!autoStarted) {
            autoStarted = true;
            follower.setStartingPose(START_POSE);
            telemetry.addData("START cu AprilTag", aprilTagFromInit);
            telemetry.update();

            SequentialCommandGroup autoSequence = new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new InstantCommand(() -> follower.setMaxPower(1)),
                        new InstantCommand(() -> carousel.setShooterForAutoRPM(3500)),
                        new PrepareOuttakeFromTagCommand(carousel, () -> this.aprilTagFromInit)
                    ),
                    new FollowPathCommand(follower, scorePreloadPath, false),
                    new ShootAllBallsCommand(carousel),



                    //--- CICLUL 2: PRIMA COLECTARE ȘI SCOR ---
                    new InstantCommand(() -> intake.setPower(-0.3)),

                    new ParallelRaceGroup(
                            //new WaitUntilCommand(carousel::allSlotsOccupied),
                            new FollowPathCommand(follower, grab1Path, false),
                            new WaitCommand(5000)
                    ),
                    /*new InstantCommand(() -> intake.setPower(0)),
                    new InstantCommand(() -> follower.setMaxPower(1)),*/
                    new InstantCommand(() -> intake.setPower(0.1)),


                    new ParallelCommandGroup(
                            new InstantCommand(() -> follower.setMaxPower(1)),
                            new InstantCommand(() -> carousel.setShooterForAutoRPM(3500)),
                            new PrepareOuttakeFromTagCommand(carousel, () -> this.aprilTagFromInit)
                    ),
                    new FollowPathCommand(follower, score1Path, false),
                    new ShootAllBallsCommand(carousel),


                    // CICLUL 3: A doua colectare si scor
                    new InstantCommand(() -> intake.setPower(-0.3)),
                    new ParallelRaceGroup(
                            new FollowPathCommand(follower, grab2Path, false),
                            new WaitCommand(5000)
                    ),
                    /*new InstantCommand(() -> intake.setPower(0)),
                    new InstantCommand(() -> follower.setMaxPower(1)),*/
                    new InstantCommand(() -> intake.setPower(0.1)),

                    new ParallelCommandGroup(
                            new InstantCommand(() -> follower.setMaxPower(1)),
                            new InstantCommand(() -> carousel.setShooterForAutoRPM(3500)),
                            new PrepareOuttakeFromTagCommand(carousel, () -> this.aprilTagFromInit)
                    ),
                    new FollowPathCommand(follower, score2Path, false),
                    new ShootAllBallsCommand(carousel),

                    // CICLU 4 a treia colectare

                   new InstantCommand(() -> intake.setPower(-0.3)),

                    new ParallelRaceGroup(
                            new FollowPathCommand(follower, grab3Path, false),
                            new WaitCommand(5000)
                    ),
                    /*new InstantCommand(() -> intake.setPower(0)),astea erau comentate
                    new InstantCommand(() -> follower.setMaxPower(1)), asta era comentata*/
                    new InstantCommand(() -> intake.setPower(0.1)),


                    new ParallelCommandGroup(
                            new InstantCommand(() -> follower.setMaxPower(1)),
                            new InstantCommand(() -> carousel.setShooterForAutoRPM(3500)),
                            new PrepareOuttakeFromTagCommand(carousel, () -> this.aprilTagFromInit)
                    ),
                    new FollowPathCommand(follower, score3Path, false),
                    new ShootAllBallsCommand(carousel),
                    new InstantCommand(() -> intake.setPower(0)),

                    new InstantCommand(() -> follower.setMaxPower(1)),
                    new FollowPathCommand(follower, parkPath, false)

            );
            schedule(autoSequence);
        }
    }
}
