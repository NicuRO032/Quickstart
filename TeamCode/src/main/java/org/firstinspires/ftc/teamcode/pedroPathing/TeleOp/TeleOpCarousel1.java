package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

//import static org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem1.velocityBeforePush;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem1;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.IntakeSubsystem1;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.VisionSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="TeleOp_Final_cu_Turela")
@Config
public class TeleOpCarousel1 extends OpMode {

    private Follower follower;
    public static Pose startingPose;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private GamepadEx driver1;
    private GamepadEx driver2;

    private CarouselSubsystem1 carousel;
    private TurretSubsystem turret;
    private VisionSubsystem vision;
    private IntakeSubsystem1 intake;

    private FtcDashboard dashboard;

    private boolean outtakePrepared = false;
    private boolean hasRumbled = false;

    private enum TurretTeleOpState { MANUAL, SEMI_AUTO_SEARCHING, SEMI_AUTO_LOCKING }
    private TurretTeleOpState turretTeleOpState = TurretTeleOpState.MANUAL;
    private final ElapsedTime lockOnTimer = new ElapsedTime();
    public static boolean intakeIsOn = false;

    public static double SHOOT_RPM = 3000, ANGLE_SHOOT = 0;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        if (startingPose == null) {
            startingPose = new Pose(0, 0, 0);
        }
        intakeIsOn = false;
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
        carousel = new CarouselSubsystem1(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
        intake = new IntakeSubsystem1(hardwareMap);

        //carousel.resetForStart();
        carousel.activateIntake();

        dashboard = FtcDashboard.getInstance();
        CommandScheduler.getInstance().registerSubsystem(carousel, turret, vision, intake);
        telemetry.addData("Analog Feedback:", "%.3f V", carousel.getCurrentFeedbackMv());
        telemetry.addLine("INIT: pentru START, PUNE SLOTUL 1 in fata cu feedback aprox. 1400");
        if (Math.abs(carousel.getCurrentFeedbackMv()-1400)>200){
            telemetry.addLine("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            telemetry.addLine("!!!!!!Pozitionare incorecta, STOP si reluati!!!!!!");
            telemetry.addLine("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        }
        else{
            telemetry.addLine("PUTEM INCEPE...");
            telemetry.addLine("Dupa START, slotul 0 va veni in fata...");

        }
        telemetry.update();

        vision.disableProcesor();
    }

    @Override
    public void start() {
        carousel.resetForStart();
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        follower.update();

        if (driver1.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            follower.setTeleOpDrive(
                        0, 0, 0, true
            );
        } else {
            if (!slowMode) follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        gamepad1.left_trigger - gamepad1.right_trigger,
                        true
            );
            else follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * slowModeMultiplier,
                        -gamepad1.left_stick_x * slowModeMultiplier,
                        (gamepad1.left_trigger - gamepad1.right_trigger) * slowModeMultiplier,
                        true
            );
        }

        if (gamepad1.rightBumperWasPressed()) {
                slowMode = !slowMode;
        }

        driver1.readButtons();
        driver2.readButtons();

        handleDriver1Controls();
        handleDriver2Controls();

        sendTelemetry();

    }

    private void handleDriver1Controls() {
        final double STICK_DEADZONE = 0.1;
        double joystickPower = driver1.getRightY() * 0.9;

        if (Math.abs(joystickPower) > STICK_DEADZONE) {
            intake.setPower(joystickPower);
            intakeIsOn = false;
        } else {
            if (driver1.wasJustPressed(GamepadKeys.Button.A)) {
                intakeIsOn = !intakeIsOn;
            }
            if (intakeIsOn) {
                intake.setPower(-0.8);
            } else {
                intake.stop();
            }
        }

        if (driver1.wasJustPressed(GamepadKeys.Button.BACK)) {
            carousel.abortAll();
            outtakePrepared = false;
            hasRumbled = false;
        }

        if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) carousel.manualStepLeft();
        if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) carousel.manualStepRight();
        if (driver1.wasJustPressed(GamepadKeys.Button.X)) carousel.setActivePattern(CarouselSubsystem1.OuttakePattern.GPP);
        if (driver1.wasJustPressed(GamepadKeys.Button.Y)) carousel.setActivePattern(CarouselSubsystem1.OuttakePattern.PGP);
        if (driver1.wasJustPressed(GamepadKeys.Button.B)) carousel.setActivePattern(CarouselSubsystem1.OuttakePattern.PPG);
        if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) && !outtakePrepared) {
            carousel.prepareOuttake(carousel.getActivePattern());
            outtakePrepared = true;
            hasRumbled = false;
            intakeIsOn = false;
        }

        if (carousel.isReadyToShoot() && !hasRumbled) {
            gamepad1.rumble(500);
            hasRumbled = true;
        }

        if (carousel.getOuttakeState().equals("OUT_IDLE")) outtakePrepared = false;
    }

    private void handleDriver2Controls() {
        // --- Shooter Angle Control ---
        //turret.setManualShooterAngle(-driver2.getLeftY());

        if(driver2.wasJustPressed(GamepadKeys.Button.Y)) turret.setShooterAngle(0.65); // Unghi pentru inaltime mica
        if(driver2.wasJustPressed(GamepadKeys.Button.B)) turret.setShooterAngle(0.5); // Unghi inaltime medie
        if(driver2.wasJustPressed(GamepadKeys.Button.A)) turret.setShooterAngle(0.0); // Unghi pentru inaltime mare


        // --- Turret Rotation and Outtake ---
        if (driver2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
            if(carousel.canSkipShoot()){
                carousel.skipTrow();
            }else
                carousel.triggerShoot();
        }

        if (driver2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            if (turretTeleOpState == TurretTeleOpState.MANUAL) {
                vision.enableProcesor();
                turretTeleOpState = TurretTeleOpState.SEMI_AUTO_SEARCHING;
            } else {
                vision.disableProcesor();
                turretTeleOpState = TurretTeleOpState.MANUAL;
                turret.setManualControl(0);
            }
        }

        AprilTagDetection bestTag = vision.getBestDetection();
        boolean hasValidTarget = (bestTag != null && bestTag.metadata != null && (bestTag.id == 20 || bestTag.id == 24));

        switch (turretTeleOpState) {
            case MANUAL:
                driver2.gamepad.setLedColor(0, 1, 0, -1);
                turret.setManualControl(-driver2.getRightX());
                if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) turret.setTargetAngle(0.0);
                if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) turret.setTargetAngle(-90.0);
                if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) turret.setTargetAngle(90.0);
                if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    AprilTagDetection currentTag = vision.getBestDetection();
                    // Verificăm dacă avem o țintă validă înainte de a comanda mișcarea
                    if (currentTag != null && currentTag.metadata != null) {
                        // Calculăm unghiul final: poziția curentă + corecția necesară
                        double targetAngle = turret.getCurrentAngle() + currentTag.ftcPose.bearing;
                        turret.setTargetAngle(targetAngle);
                    }
                }
                break;

            case SEMI_AUTO_SEARCHING:
                driver2.gamepad.setLedColor(1, 0, 0, -1);
                if(carousel.canChangeRPM()){
                    double x = vision.getDistance();
                    carousel.setShooterTargetRPM(-0.0302055 * x * x  + 30.9530 * x + 2400.77337);
                    turret.setShooterAngle(-0.0000824054 * x * x + 0.0132998 * x -0.184169);
//                    carousel.setShooterTargetRPM(SHOOT_RPM);
//                    turret.setShooterAngle(ANGLE_SHOOT);
                }
                if (hasValidTarget) {
                    turretTeleOpState = TurretTeleOpState.SEMI_AUTO_LOCKING;
                    driver2.gamepad.rumble(50); //am miscorat valoarea ca sa nu blochez sistemul
                    lockOnTimer.reset();
                    turret.commandAutoAim(bestTag);
                } else {
                    turret.setManualControl(-driver2.getRightX());
                }
                break;

            case SEMI_AUTO_LOCKING:
                driver2.gamepad.setLedColor(1, 0, 0, -1);
                if(carousel.canChangeRPM()){
                    double x = vision.getDistance();
                    carousel.setShooterTargetRPM(-0.0302055 * x * x  + 30.9530 * x + 2400.77337);
                    turret.setShooterAngle(-0.0000824054 * x * x + 0.0132998 * x -0.184169);
//                    carousel.setShooterTargetRPM(SHOOT_RPM);
//                    turret.setShooterAngle(ANGLE_SHOOT);
                }
                if (hasValidTarget) {
                    turret.commandAutoAim(bestTag);
                    double bearingError = bestTag.ftcPose.bearing;

                    if (Math.abs(bearingError) < TurretSubsystem.AIMING_TOLERANCE_DEGREES) {
                        if (lockOnTimer.milliseconds() > 500) {
                            driver2.gamepad.rumble(0.7, 0.7, 200);
                        }
                    } else {
                        lockOnTimer.reset();
                    }
                } else {
                    turretTeleOpState = TurretTeleOpState.SEMI_AUTO_SEARCHING;
                    turret.setManualControl(0);
                }
                break;
        }
        //sendTelemetry();
    }

    @SuppressLint("DefaultLocale")
    private void sendTelemetry() {
        AprilTagDetection bestTag = vision.getBestDetection();
        double bearing = (bestTag != null && bestTag.ftcPose != null) ? bestTag.ftcPose.bearing : 0.0;

        telemetry.addLine("--- TURELA ---");
        telemetry.addData("TeleOp State", turretTeleOpState);
        telemetry.addData("Subsystem State", turret.getControlState());
        telemetry.addData("Target Angle", "%.1f", turret.getTargetAngle());
        telemetry.addData("Current Angle", "%.1f", turret.getCurrentAngle());
        telemetry.addData("AprilTag Bearing", "%.1f", bearing);
        telemetry.addLine("\n--- SHOOTER ANGLE ---");
        telemetry.addData("Angle Control Mode", turret.getAngleControlState());
        telemetry.addData("Shooter Angle Position", "%.2f", turret.getShooterAnglePosition());
        telemetry.addLine("\n--- CARUSEL ---");
        telemetry.addData("Outtake State", carousel.getOuttakeState());
        telemetry.addLine("\n--- SHOOTER VELOCITY RPM---");
        telemetry.addData("Target Velo", "%.2f", carousel.getShooterTargetRPM());
        telemetry.addData("Current Velo", "%.2f", carousel.getShooterCurrentRPM());
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        /**
        packet.put("01. Turret TeleOp State", turretTeleOpState.name());
        packet.put("02. Turret Subsystem State", turret.getControlState().name());
        packet.put("03. Turret Target", turret.getTargetAngle());
        packet.put("04. Turret Current", turret.getCurrentAngle());
        packet.put("05. AprilTag Bearing", bearing);
        packet.put("06. Shooter Angle Mode", turret.getAngleControlState());
        packet.put("07. Shooter Angle Pos", turret.getShooterAnglePosition());
        packet.put("08. Global index", carousel.getGlobalIndex());
        packet.put("09. LogicalIndex", carousel.getLogicalIndex());
        packet.put("10. Outtake State", carousel.getOuttakeState());
**/

        packet.put("00. Intake State", carousel.getIntakeState());
        packet.put("10. Outtake State", carousel.getOuttakeState());
        packet.put("02.LogicalIndex", carousel.getLogicalIndex());

        packet.put("040.Carousel Target Feedback (mV)", carousel.getTargetFeedbackMv());
        packet.put("041.Carousel Current Feedback (mV)", carousel.getCurrentFeedbackMv());
        packet.put("042.Carousel Feedback Error (mV)", carousel.getFeedbackError());
        packet.put("043.Carousel At Target", carousel.atTarget()); // Foarte util de monitorizat


        packet.put("050. Main Distance (mm)", carousel.getMainDistance());
        packet.put("051. Color1 Distance (mm)", String.format("%.3f", carousel.getColor1Distance()));
        packet.put("052. Color2 Distance (mm)", String.format("%.3f", carousel.getColor2Distance()));
        packet.put("053. Entry Slot Has Ball", carousel.entrySlotHasBall());
        packet.put("06.Occupied 0", carousel.getOccupied(0));
        packet.put("07.Occupied 1", carousel.getOccupied(1));
        packet.put("08.Occupied 2", carousel.getOccupied(2));
        packet.put("09.Hue1", carousel.getHue1());
        packet.put("10.Hue2", carousel.getHue2());
        packet.put("11.HueMax", carousel.getHueMax());
        packet.put("12.Slot colors", carousel.getSlotsColorString());
        packet.put("13.Outtake order string", carousel.getOuttakeOrderString());
        packet.put("14.Distance: ", vision.getDistance());
        packet.put("15.X:", vision.getLastX());
        packet.put("16.Y:", vision.getLastY());
        packet.put("17.Angle:", turret.getCurrentShooterAngle());

        // Adaugă telemetria pentru viteza shooter-ului aici
        packet.put("Shooter Target Velocity", carousel.getShooterTargetRPM());
        packet.put("Shooter Current Velocity", carousel.getShooterCurrentRPM());
        packet.put("Velocity before push", carousel.getRpmBeforePush());

/**
        packet.put("14.Turret TeleOp State", turretTeleOpState.name());
        packet.put("15.Turret Subsystem State", turret.getControlState().name());
        packet.put("16.Turret Target", turret.getTargetAngle());
        packet.put("17.Turret Current", turret.getCurrentAngle());
        packet.put("18.AprilTag Bearing", bearing);
**/
        dashboard.sendTelemetryPacket(packet);
    }
}
