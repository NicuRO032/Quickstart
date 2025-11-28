package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

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

import org.firstinspires.ftc.teamcode.pedroPathing.Commands.RunOuttakeSequenceCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.OuttakeSubsystem;

import java.util.function.Supplier;


@Configurable
@TeleOp(name="ExampleTeleOpOK")
public class ExampleTeleOpOK extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private IntakeSubsystem intake;
    private OuttakeSubsystem outtake;


    public boolean sequenceRunning = false;


    @Override
    public void init() {


        // Initialize subsystems
        intake = new IntakeSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);

        // Register subsystems in the CommandScheduler
        CommandScheduler.getInstance().registerSubsystem(intake);
        CommandScheduler.getInstance().registerSubsystem(outtake);

        telemetry.addLine("Initialized Command TeleOp with Scheduler");
        telemetry.update();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        outtake.setShooterServoPos(0.9);
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();
        CommandScheduler.getInstance().run();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    gamepad1.left_trigger - gamepad1.right_trigger,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    (gamepad1.left_trigger - gamepad1.right_trigger) * slowModeMultiplier,
                    true // Robot Centric
            );
        }

        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }

        //Optional way to change slow mode strength
        if (gamepad2.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }

        if(!sequenceRunning) {
            intake.setBrushMotorPower(-gamepad1.right_stick_y);
        }
        //CommandScheduler.getInstance().schedule(new RunIntakeBrushCommand(intake, -gamepad1.right_stick_y));
        //CommandScheduler.getInstance().schedule(new RunIntakeBeltCommand(intake, -gamepad1.right_stick_y));

        /**
        if (gamepad1.y){
         CommandScheduler.getInstance().schedule(new RunOuttakeServoPosCommand(outtake, SERVO_HOLD));
         }
         if (gamepad1.x){
             CommandScheduler.getInstance().schedule(new RunOuttakeServoPosCommand(outtake, SERVO_FIRE));
         }
        **/

        if (gamepad1.y){
            sequenceRunning = true;
            CommandScheduler.getInstance().schedule(
                    new RunOuttakeSequenceCommand(intake, outtake).andThen(
                            new InstantCommand(() -> sequenceRunning = false))
            );
            //CommandScheduler.getInstance().schedule(new OuttakeSequence(intake, outtake));
        }



        /**if (gamepad1.y){
            CommandScheduler.getInstance().schedule(new RunOuttakeShooterCommand(outtake, 0.25));
        }
        if (gamepad1.x){
            CommandScheduler.getInstance().schedule(new RunOuttakeShooterCommand(outtake, 0.0));
        }**/

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}