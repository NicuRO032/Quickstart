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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem1;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.IntakeSubsystem1;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.VisionSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="TeleOp_Final_cu_Turela")
@Config
public class TeleOpCarousel1 extends OpMode {
    //private IntakeSubsystem1 intake1;

    private enum Alliance { BLUE, RED, UNKNOWN }
    private Alliance selectedAlliance = Alliance.UNKNOWN;
    private int targetAprilTagId = 0; // ID-ul țintei, 0 înseamnă niciuna

    private Follower follower;
    public static Pose startingPose;
    private boolean slowMode = false;
    public static double slowModeMultiplier = 0.5;

    private boolean slowShoot = false, fastShoot = false;
    private ElapsedTime delayAruncare = new ElapsedTime();


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
    //public static boolean Intake1IsOn = false;

    //folosim pentru vibrat maneta daca avem 3 bile in carusel
    private boolean prevAllBalls = false, curAllBalls = false;

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
        CommandScheduler.getInstance().registerSubsystem(carousel, turret, vision, intake, intake);

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
        telemetry.addLine("NUMA BILE!");
        telemetry.addLine("========================================");
        telemetry.update();

        //vision.disableProcesor();
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
            gamepad1.rumble(200);
        }

        curAllBalls = carousel.allSlotsOccupied();

        if(curAllBalls && !prevAllBalls){
            gamepad1.rumbleBlips(3);
        }

        prevAllBalls = curAllBalls;

        driver1.readButtons();
        driver2.readButtons();

        handleDriver1Controls();
        handleDriver2Controls();

        //sendTelemetryMatches();
        sendTelemetry();
        //sendTelemetryMotorsCurrent();

    }

    private void handleDriver1Controls() {
        final double STICK_DEADZONE = 0.1;
        double joystickPower = driver1.getRightY();

        if (Math.abs(joystickPower) > STICK_DEADZONE) {
            // Control manual (Override): Folosește metoda cu protecție electrică
            intake.setPower(joystickPower);
            intakeIsOn = false;
        } else {
            // Logica butonului A (Toggle)
            if (driver1.wasJustPressed(GamepadKeys.Button.A)) {
                // Nu permitem activarea dacă robotul este deja plin (prevenire penalty)
                if (!carousel.allSlotsOccupied()) {
                    intakeIsOn = !intakeIsOn;
                } else {
                    intakeIsOn = false;
                    gamepad1.rumble(200); // Feedback că e plin
                }
            }

            // Obținem starea mașinii de stări de intake din carusel
            CarouselSubsystem1.IntakeState currentIntakeState = carousel.getIntakeStateEnum();

            // Controlăm motoarele DOAR dacă caruselul este în IDLE
            // Dacă e în STORE_AND_ADVANCE sau CLEANUP_EXCESS, lăsăm automatizarea să lucreze
            if (currentIntakeState == CarouselSubsystem1.IntakeState.IDLE) {
                if (intakeIsOn) {
                    intake.collect(); // Folosește starea COLLECTING (ambele motoare -0.8)
                } else {
                    intake.stop();    // Folosește starea IDLE (0.0 cu protecție)
                }
            }
        }

        // --- Restul butoanelor rămân la fel ---
        if (driver1.wasJustPressed(GamepadKeys.Button.BACK)) {
            carousel.abortAll();
            outtakePrepared = false;
            hasRumbled = false;
            intakeIsOn = false; // Resetăm și variabila de control
        }

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
        // --- Ridicam un steag pentru aruncare, driverul 2 e gata sa arunce oricand robotul este ---
        // --- Daca trec mai mult de 5 secunde consideram apasare accidentala si nu mai aruncam ---
        if (driver2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            fastShoot = true;
            delayAruncare.reset();
        }
        if (driver2.wasJustPressed(GamepadKeys.Button.X)) {
            slowShoot = true;
            delayAruncare.reset();
        }
        if(carousel.getReadyToShootCarousel() && delayAruncare.seconds() < 5) {
            if(fastShoot){
                fastShoot = false;
                slowShoot = false;
                carousel.triggerShoot();
            }
            if(slowShoot){
                slowShoot = false;
                fastShoot = false;
                carousel.triggerSlowShoot();

            }
        }


        // --- Comutare Mod Turelă (Manual <-> Auto-Aim) ---
        if (driver2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            if (turretTeleOpState == TurretTeleOpState.MANUAL) {
                // Permitem trecerea la auto-aim DOAR dacă o alianță a fost selectată în init()
                if (selectedAlliance != Alliance.UNKNOWN) {
                    //vision.enableProcesor();
                    turretTeleOpState = TurretTeleOpState.SEMI_AUTO_SEARCHING;
                }
            } else {
                // Ieșim din modul auto-aim și ne întoarcem la manual
                //vision.disableProcesor();
                turretTeleOpState = TurretTeleOpState.MANUAL;
                turret.setManualControl(0); // Oprește mișcarea turelei la ieșirea din mod
            }
        }


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
                /**
                if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    AprilTagDetection currentTag = vision.getBestDetection();
                    if (currentTag != null && currentTag.metadata != null) {
                        double targetAngle = turret.getCurrentAngle() - currentTag.ftcPose.bearing;
                        turret.setTargetAngle(targetAngle);
                    }
                }
                 **/
                break;

            case SEMI_AUTO_SEARCHING:
                driver2.gamepad.setLedColor(1, 0, 0, -1); // Roșu pentru Căutare

                // Permite controlul manual cu joystick-ul PÂNĂ când ținta este găsită
                turret.setManualControl(-driver2.getRightX());

                // Setarea RPM-ului și unghiului în funcție de distanță (păstrată)
                if (vision.hasValidTag() && carousel.canChangeRPM()) { // Verificăm dacă avem o țintă vizibilă, chiar dacă nu e cea corectă
                    double x = vision.getDistance();
                    carousel.setShooterTargetRPM(5.82256 * x + 2597.00876);
                    turret.setShooterAngle(0.000952381 * x - 0.00555556);
                    //carousel.setShooterTargetRPM(SHOOT_RPM);
                    //turret.setShooterAngle(ANGLE_SHOOT);
                }

                // Dacă am găsit ȚINTA CORECTĂ, trecem la LOCKING
                if (vision.hasValidTag()) {
                    turretTeleOpState = TurretTeleOpState.SEMI_AUTO_LOCKING;
                    driver2.gamepad.rumble(50);
                    lockOnTimer.reset();
                    // NOU: Apelăm commandAutoAim cu ID-ul țintei
                    turret.commandAutoAim(
                            vision.hasValidTag(),
                            vision.getLastTagId(),
                            vision.getLastBearing(),
                            targetAprilTagId
                    );
                }
                break;

            case SEMI_AUTO_LOCKING:
                // ▼▼▼ AICI ESTE MODIFICAREA PRINCIPALĂ ▼▼▼
                // Verificăm MAI ÎNTÂI dacă avem o țintă validă.
                if (vision.hasValidTag()) {
                    // ---- Dacă avem țintă, executăm toată logica de ochire și feedback ----

                    // 1. Schimbăm culoarea LED-ului în funcție de precizie
                    double bearingError = vision.getLastBearing();
                    if (Math.abs(bearingError) < TurretSubsystem.AIMING_TOLERANCE_DEGREES) {
                        driver2.gamepad.setLedColor(255, 0, 255, -1); // Albastru pentru Lock-On reușit
                    } else {
                        driver2.gamepad.setLedColor(1, 0.5, 0, -1); // Portocaliu pentru Ajustare
                    }

                    // 2. Setăm RPM-ul și unghiul shooter-ului în funcție de distanță
                    if (carousel.canChangeRPM()) {
                        double x = vision.getDistance();
                        carousel.setShooterTargetRPM(5.82256 * x + 2597.00876);
                        turret.setShooterAngle(0.000952381 * x - 0.00555556);
                        //carousel.setShooterTargetRPM(SHOOT_RPM);
                        //turret.setShooterAngle(ANGLE_SHOOT);
                    }

                    // 3. Comandăm turelei să continue ochirea
                    turret.commandAutoAim(
                            vision.hasValidTag(),
                            vision.getLastTagId(),
                            vision.getLastBearing(),
                            targetAprilTagId
                    );

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
        double bearing = vision.getLastBearing();

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

        // --- Secțiunea 00-02: Stări și Indecși (String/Int - nu au nevoie de clean) ---
        packet.put("00. Intake State", carousel.getIntakeState());
        packet.put("10. Outtake State", carousel.getOuttakeState());
        packet.put("101.SlowShootState", carousel.getSlowShootState());
        packet.put("02.LogicalIndex", carousel.getLogicalIndex());

        // --- Secțiunea 04: Feedback Carusel (3 zecimale) ---
        packet.put("040.Carousel Target Feedback (mV)", clean(carousel.getTargetFeedbackMv()));
        packet.put("041.Carousel Current Feedback (mV)", clean(carousel.getCurrentFeedbackMv()));
        packet.put("042.Carousel Feedback Error (mV)", clean(carousel.getFeedbackError()));
        packet.put("043.Carousel At Target", carousel.atTarget());

        // --- Secțiunea 05: Senzori Distanță și Culori ---
        packet.put("050. Slot Distance (mm)", clean(carousel.getSlotDistance()));
        packet.put("053. Entry Slot Has Ball", carousel.entrySlotHasBall());
        packet.put("054. Gate Distance (mm)", clean(carousel.getGateDistance()));
        packet.put("06.Occupied 0", carousel.getOccupied(0));
        packet.put("07.Occupied 1", carousel.getOccupied(1));
        packet.put("08.Occupied 2", carousel.getOccupied(2));

        // --- Secțiunea 14-18: Vision și Turelă (Aici erau numerele mari) ---
        packet.put("14.Distance: ", clean(vision.getDistance()));
        packet.put("15.X Offset", clean(vision.getLastX()));
        packet.put("16.Y Offset", clean(vision.getLastY()));
        packet.put("17.Shooter Angle Pos", clean(turret.getCurrentShooterAngle()));
        packet.put("18.Turret Target Angle", clean(turret.getTargetAngle()));
        packet.put("18.AprilTag Bearing", clean(vision.getLastBearing()));

        // --- Secțiunea 20-24: Status Shooter ---
        packet.put("20 Carousel Target feedback", clean(carousel.getTargetFeedbackMv()));
        packet.put("21 Carousel Current feedback", clean(carousel.getCurrentFeedbackMv()));
        packet.put("22 Carousel Feedback Error", clean(carousel.getFeedbackError()));
        packet.put("23 Carousel At Target", carousel.atTarget());
        packet.put("24 IsShooterReady", carousel.isShooterReady());
        packet.put("04 IsReadyToShoot", carousel.isReadyToShoot());

        // --- Viteze Shooter ---
        packet.put("Shooter Target Velocity", clean(carousel.getShooterTargetRPM()));
        packet.put("Shooter Current Velocity", clean(carousel.getShooterCurrentRPM()));
        packet.put("Shooter Power", clean(carousel.getShooterPower()));


/**
 packet.put("14.Turret TeleOp State", turretTeleOpState.name());
 packet.put("15.Turret Subsystem State", turret.getControlState().name());
 packet.put("16.Turret Target", turret.getTargetAngle());
 packet.put("17.Turret Current", turret.getCurrentAngle());
 packet.put("18.AprilTag Bearing", bearing);
 **/
        dashboard.sendTelemetryPacket(packet);
    }

    private void sendTelemetryMotorsCurrent() {
        // --- Citire Motoare Drivetrain ---
        // Folosim numele din Constants.java: "rf", "rr", "lr", "lf"
        DcMotorEx mRF = hardwareMap.get(DcMotorEx.class, "rf");
        DcMotorEx mRR = hardwareMap.get(DcMotorEx.class, "rr");
        DcMotorEx mLF = hardwareMap.get(DcMotorEx.class, "lf");
        DcMotorEx mLR = hardwareMap.get(DcMotorEx.class, "lr");

        double currentRF = mRF.getCurrent(CurrentUnit.AMPS);
        double currentRR = mRR.getCurrent(CurrentUnit.AMPS);
        double currentLF = mLF.getCurrent(CurrentUnit.AMPS);
        double currentLR = mLR.getCurrent(CurrentUnit.AMPS);
        double totalCurrent = currentRF + currentRR + currentLF + currentLR;

        // --- Afișare în Telemetria de pe Driver Station ---
        telemetry.addLine("\n--- DRIVETRAIN CURRENT (Amps) ---");
        telemetry.addData("Total Drivetrain", "%.2f A", totalCurrent);
        telemetry.addData("FL / FR", "%.2f A | %.2f A", currentLF, currentRF);
        telemetry.addData("RL / RR", "%.2f A | %.2f A", currentLR, currentRR);

        // --- Afișare în FTC Dashboard (Grafice) ---
        TelemetryPacket packet = new TelemetryPacket();
        // ... codul tău existent pentru packet.put ...

        packet.put("Drivetrain Total Amps", String.format("%.3f", totalCurrent));
        packet.put("Motor RF Amps", String.format("%.3f", currentRF));
        packet.put("Motor RR Amps", String.format("%.3f", currentRR));
        packet.put("Motor LF Amps", String.format("%.3f", currentLF));
        packet.put("Motor LR Amps", String.format("%.3f", currentLR));

        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }

    private void sendTelemetryMatches(){
        telemetry.addData("Slow mode: ", slowMode);
        telemetry.addData("Bile ", carousel.getNoBalls());
    }

    private double clean(double val) {
        if (Double.isNaN(val) || Double.isInfinite(val)) return 0;
        return Math.round(val * 1000.0) / 1000.0;
    }
}
