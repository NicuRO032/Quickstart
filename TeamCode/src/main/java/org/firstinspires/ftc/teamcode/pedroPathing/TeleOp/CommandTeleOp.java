package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.seattlesolvers.solverslib.command.CommandScheduler;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Commands.RunIntakeCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Commands.RunOuttakeCommand;

@TeleOp(name = "Command TeleOp (Subsystems + Scheduler)", group = "Pedro Pathing")
public class CommandTeleOp extends OpMode {

    private Follower follower;
    private IntakeSubsystem intake;
    private OuttakeSubsystem outtake;

    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    @Override
    public void init() {
        // Initialize robot drive and pose follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        follower.update();

        // Initialize subsystems
        intake = new IntakeSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);

        // Register subsystems in the CommandScheduler
        CommandScheduler.getInstance().registerSubsystem(intake);
        CommandScheduler.getInstance().registerSubsystem(outtake);

        telemetry.addLine("Initialized Command TeleOp with Scheduler");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        CommandScheduler.getInstance().run();

        // === DRIVING CONTROL ===
        double driveY = -gamepad1.left_stick_y;
        double driveX = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        if (slowMode) {
            driveY *= slowModeMultiplier;
            driveX *= slowModeMultiplier;
            turn *= slowModeMultiplier;
        }

        follower.setTeleOpDrive(driveY, driveX, turn, true);

        // Toggle slow mode
        if (gamepad1.right_bumper) {
            slowMode = !slowMode;
        }

        // === INTAKE CONTROL ===
        if (gamepad2.right_trigger > 0.2) {
            CommandScheduler.getInstance().schedule(new RunIntakeCommand(intake, 1.0));
        } else if (gamepad2.left_trigger > 0.2) {
            CommandScheduler.getInstance().schedule(new RunIntakeCommand(intake, -1.0));
        } else {
            intake.stop();
        }

        // === OUTTAKE CONTROL ===
        if (gamepad2.a) {
            CommandScheduler.getInstance().schedule(new RunOuttakeCommand(outtake, 0.8, 1.0)); // motor + servo deschis
        } else if (gamepad2.b) {
            CommandScheduler.getInstance().schedule(new RunOuttakeCommand(outtake, -0.5, 0.0)); // motor invers + servo jos
        } else {
            outtake.stopMotor();
        }

        // === TELEMETRY ===
        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Outtake Motor Power", outtake.getPower());
        telemetry.addData("Outtake Servo Pos", outtake.getServoPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
        intake.stop();
        outtake.stopMotor();
    }
}
