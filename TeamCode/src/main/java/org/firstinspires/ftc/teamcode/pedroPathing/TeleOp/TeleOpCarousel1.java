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
    private enum Alliance { BLUE, RED, UNKNOWN }
    private Alliance selectedAlliance = Alliance.UNKNOWN;
    private int targetAprilTagId = 0; // ID-ul țintei, 0 înseamnă niciuna

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
        intake = new IntakeSubsystem1(hardwareMap);
        carousel = new CarouselSubsystem1(hardwareMap, intake);
        //carousel = new CarouselSubsystem1(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
        //intake = new IntakeSubsystem1(hardwareMap);
        carousel.isTeleOp = true;

        //carousel.resetForStart();
        carousel.activateIntake();

        dashboard = FtcDashboard.getInstance();
        CommandScheduler.getInstance().registerSubsystem(carousel, turret, vision, intake);

        // Afișăm instrucțiunile o singură dată dacă alianța nu a fost încă selectată
        if (selectedAlliance == Alliance.UNKNOWN) {
            telemetry.addLine(">>> ALEGE ALIANTA <<<");
            telemetry.addLine("Apasa 'X' (Gamepad 1 sau 2) pentru ALBASTRU");
            telemetry.addLine("Apasa 'B' (Gamepad 1 sau 2) pentru ROSU");
        }

        // Verificăm apăsările de butoane în fiecare ciclu al buclei de init
        if (gamepad1.x || gamepad2.x) {
            if (selectedAlliance != Alliance.BLUE) {
                selectedAlliance = Alliance.BLUE;
                targetAprilTagId = 20; // ID-ul pentru turnul albastru
                gamepad1.rumble(250);
                gamepad2.rumble(250);
            }
        }
        if (gamepad1.b || gamepad2.b) {
            if (selectedAlliance != Alliance.RED) {
                selectedAlliance = Alliance.RED;
                targetAprilTagId = 24; // ID-ul pentru turnul roșu
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            }
        }

        // Actualizăm telemetria pentru a arăta starea curentă
        telemetry.addData("ALIANTĂ SELECTATĂ", selectedAlliance);
        telemetry.addData("ID AprilTag Țintă", targetAprilTagId);

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


        telemetry.addLine("========================================");
        telemetry.addData("ALIANTĂ SELECTATĂ", selectedAlliance);
        telemetry.addData("ID AprilTag Țintă", targetAprilTagId);
        telemetry.addLine("GATA DE START!");
        telemetry.addLine("========================================");
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

            // Obținem starea mașinii de stări de intake din carusel
            CarouselSubsystem1.IntakeState currentIntakeState = carousel.getIntakeStateEnum();

            // Acționăm asupra motorului DOAR dacă caruselul este în starea IDLE (așteptare).
            // Astfel, nu interferăm cu stările STORE_AND_ADVANCE sau REVERSE_INTAKE.
            if (currentIntakeState == CarouselSubsystem1.IntakeState.IDLE) {
                if (intakeIsOn) {
                    intake.setPower(-0.8); // Pornește intake-ul la comanda șoferului
                } else {
                    intake.stop(); // Oprește intake-ul la comanda șoferului
                }
            }
            // Dacă starea NU este IDLE, înseamnă că subsistemul Carousel are controlul.
            // Nu facem nimic și îl lăsăm să-și termine treaba (ex: să ruleze în marșarier).
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
        // --- Control Unghi Shooter (păstrat) ---
        if (driver2.wasJustPressed(GamepadKeys.Button.Y))
            turret.setShooterAngle(0.37); // Unghi pentru inaltime mica
        if (driver2.wasJustPressed(GamepadKeys.Button.B))
            turret.setShooterAngle(0.15); // Unghi inaltime medie
        if (driver2.wasJustPressed(GamepadKeys.Button.A))
            turret.setShooterAngle(0.06); // Unghi pentru inaltime mare

        // --- Control Outtake (păstrat) ---
        if (driver2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            carousel.triggerShoot();
        }

        // --- Comutare Mod Turelă (Manual <-> Auto-Aim) ---
        if (driver2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            if (turretTeleOpState == TurretTeleOpState.MANUAL) {
                // Permitem trecerea la auto-aim DOAR dacă o alianță a fost selectată în init()
                if (selectedAlliance != Alliance.UNKNOWN) {
                    vision.enableProcesor();
                    turretTeleOpState = TurretTeleOpState.SEMI_AUTO_SEARCHING;
                }
            } else {
                // Ieșim din modul auto-aim și ne întoarcem la manual
                vision.disableProcesor();
                turretTeleOpState = TurretTeleOpState.MANUAL;
                turret.setManualControl(0); // Oprește mișcarea turelei la ieșirea din mod
            }
        }

        AprilTagDetection bestTag = vision.getBestDetection();
        // NOU: 'hasValidTarget' verifică acum și ID-ul țintei selectate la inițializare
        boolean hasValidTarget = (bestTag != null && bestTag.metadata != null && bestTag.id == targetAprilTagId);

        switch (turretTeleOpState) {
            case MANUAL:
                driver2.gamepad.setLedColor(0, 1, 0, -1); // Verde pentru control Manual

                // Control manual cu joystick-ul (păstrat)
                turret.setManualControl(-driver2.getRightX());

                // Comenzi rapide manuale cu DPAD (păstrate)
                if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) turret.setTargetAngle(0.0);
                if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
                    turret.setTargetAngle(-90.0);
                if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
                    turret.setTargetAngle(90.0);

                // Comanda de ochire manuală la ținta vizibilă (păstrată)
                // Aceasta va ochi orice tag vizibil, indiferent de alianță. Util pentru testare.
                if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    AprilTagDetection currentTag = vision.getBestDetection();
                    if (currentTag != null && currentTag.metadata != null) {
                        double targetAngle = turret.getCurrentAngle() - currentTag.ftcPose.bearing;
                        turret.setTargetAngle(targetAngle);
                    }
                }
                break;

            case SEMI_AUTO_SEARCHING:
                driver2.gamepad.setLedColor(1, 0, 0, -1); // Roșu pentru Căutare

                // Permite controlul manual cu joystick-ul PÂNĂ când ținta este găsită
                turret.setManualControl(-driver2.getRightX());

                // Setarea RPM-ului și unghiului în funcție de distanță (păstrată)
                if (bestTag != null && carousel.canChangeRPM()) { // Verificăm dacă avem o țintă vizibilă, chiar dacă nu e cea corectă
                    double x = vision.getDistance();
                    carousel.setShooterTargetRPM(29.47267 * x+2624.43592);
                    turret.setShooterAngle(0.0057047 * x-0.107383);
//                    carousel.setShooterTargetRPM(SHOOT_RPM);
//                    turret.setShooterAngle(ANGLE_SHOOT);
                }

                // Dacă am găsit ȚINTA CORECTĂ, trecem la LOCKING
                if (hasValidTarget) {
                    turretTeleOpState = TurretTeleOpState.SEMI_AUTO_LOCKING;
                    driver2.gamepad.rumble(50);
                    lockOnTimer.reset();
                    // NOU: Apelăm commandAutoAim cu ID-ul țintei
                    turret.commandAutoAim(bestTag, targetAprilTagId);
                }
                break;

            case SEMI_AUTO_LOCKING:
                // ▼▼▼ AICI ESTE MODIFICAREA PRINCIPALĂ ▼▼▼
                // Verificăm MAI ÎNTÂI dacă avem o țintă validă.
                if (hasValidTarget) {
                    // ---- Dacă avem țintă, executăm toată logica de ochire și feedback ----

                    // 1. Schimbăm culoarea LED-ului în funcție de precizie
                    double bearingError = bestTag.ftcPose.bearing;
                    if (Math.abs(bearingError) < TurretSubsystem.AIMING_TOLERANCE_DEGREES) {
                        driver2.gamepad.setLedColor(0, 0, 1, -1); // Albastru pentru Lock-On reușit
                    } else {
                        driver2.gamepad.setLedColor(1, 0.5, 0, -1); // Portocaliu pentru Ajustare
                    }

                    // 2. Setăm RPM-ul și unghiul shooter-ului în funcție de distanță
                    if (carousel.canChangeRPM()) {
                        double x = vision.getDistance();
                        carousel.setShooterTargetRPM(29.47267 * x+2624.43592);
                        turret.setShooterAngle(0.0057047 * x-0.107383);
//                        carousel.setShooterTargetRPM(SHOOT_RPM);
//                        turret.setShooterAngle(ANGLE_SHOOT);
                    }

                    // 3. Comandăm turelei să continue ochirea
                    turret.commandAutoAim(bestTag, targetAprilTagId);

                    // 4. Activăm rumble-ul dacă suntem pe țintă de suficient timp
                    if (Math.abs(bearingError) < TurretSubsystem.AIMING_TOLERANCE_DEGREES) {
                        if (lockOnTimer.milliseconds() > 100) { // Am văzut că ai modificat la 100ms
                            driver2.gamepad.rumble(0.7, 0.7, 200);
                        }
                    } else {
                        // Dacă am ieșit din toleranță, resetăm cronometrul
                        lockOnTimer.reset();
                    }

                } else {
                    // ---- Dacă am pierdut ținta (bestTag este null sau are ID greșit) ----

                    // Trecem înapoi la starea de căutare
                    turretTeleOpState = TurretTeleOpState.SEMI_AUTO_SEARCHING;
                    // Oprim mișcarea automată a turelei. Acum poate fi controlată manual cu joystick-ul.
                    turret.setManualControl(0);
                    // Setăm LED-ul pe roșu pentru a indica starea de căutare
                    driver2.gamepad.setLedColor(1, 0, 0, -1);
                }
                break;

        }
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

        packet.put("040.Carousel Target Feedback (mV)", String.format("%.3f", carousel.getTargetFeedbackMv()));
        packet.put("041.Carousel Current Feedback (mV)", String.format("%.3f", carousel.getCurrentFeedbackMv()));
        packet.put("042.Carousel Feedback Error (mV)", String.format("%.3f", carousel.getFeedbackError()));
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
        packet.put("14.Distance: ", vision.getDistance());
        packet.put("15.X:", vision.getLastX());
        packet.put("16.Y:", vision.getLastY());
        packet.put("17.Shooter Angle:", turret.getCurrentShooterAngle());
        packet.put("18.Turret Angle:", turret.getTargetAngle());
        packet.put("18.AprilTag Bearing:", vision.getLastBearing());


        // Adaugă telemetria pentru viteza shooter-ului aici
        packet.put("Shooter Target Velocity", carousel.getShooterTargetRPM());
        packet.put("Shooter Current Velocity", carousel.getShooterCurrentRPM());


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
