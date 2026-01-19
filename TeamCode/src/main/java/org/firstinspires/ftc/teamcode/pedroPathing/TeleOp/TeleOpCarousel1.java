package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
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
public class TeleOpCarousel1 extends OpMode {

    private Follower follower;
    private boolean driveSystemWorking = false; // Flag pentru a ști dacă avem roți
    public static Pose startingPose;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
/// //////////////////
    private GamepadEx driver1;
    private GamepadEx driver2;

    private CarouselSubsystem1 carousel;
    private TurretSubsystem turret;
    private VisionSubsystem vision;
    private IntakeSubsystem1 intake;

    private FtcDashboard dashboard;

    // Stări pentru carusel
    private boolean outtakePrepared = false;
    private boolean hasRumbled = false;

    // Mașină de stări pentru controlul turelei din TeleOp
    private enum TurretTeleOpState { MANUAL, SEMI_AUTO_SEARCHING, SEMI_AUTO_LOCKING }
    private TurretTeleOpState turretTeleOpState = TurretTeleOpState.MANUAL;
    private final ElapsedTime lockOnTimer = new ElapsedTime();
    private boolean intakeIsOn = false;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
// 1. Asigură-te că `startingPose` nu e null
        if (startingPose == null) {
            startingPose = new Pose(0, 0, 0); // Poziție de start default
        }

        // 2. Construiește obiectul Follower. Acesta va inițializa hardware-ul.
        follower = Constants.createFollower(hardwareMap);

        // 3. DOAR DUPĂ construcție, setează poziția de start.
        //    Acest apel este esențial pentru a sincroniza starea internă a bibliotecii
        //    cu poziția ta dorită (fie cea din autonomie, fie cea default).
        follower.setStartingPose(startingPose);

        // 4. Apelează update() pentru a procesa starea inițială.
        //    Este posibil ca acest apel să nu fie necesar aici, dar nu ar trebui să strice.
        //    Dacă eroarea persistă, încearcă să comentezi linia de mai jos.
        //follower.update();


        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
        carousel = new CarouselSubsystem1(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
        intake = new IntakeSubsystem1(hardwareMap);

        carousel.resetForStart();
        carousel.activateIntake();

        dashboard = FtcDashboard.getInstance();
        CommandScheduler.getInstance().registerSubsystem(carousel, turret, vision, intake);

        telemetry.addLine("INIT: gata de START...");
        telemetry.update();
        carousel.jogServoPos(CarouselSubsystem1.JOG_OFF_POS);
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
        // --- LOGICA DE MIȘCARE (Doar dacă sistemul e ONLINE) ---
        follower.update(); // Pedro Pathing update loop
        CommandScheduler.getInstance().run();

            // Luăm input-urile de la gamepad (convertite pentru field centric sau robot centric)
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.right_stick_y,
                    -gamepad1.right_stick_x,
                    gamepad1.left_trigger - gamepad1.right_trigger,
                    true // true = Robot Centric, false = Field Centric (dacă ai localizare)
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.right_stick_y * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    (gamepad1.left_trigger - gamepad1.right_trigger) * slowModeMultiplier,
                    true // true = Robot Centric, false = Field Centric (dacă ai localizare)
            );

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }
        // -------------------------------------------------------

       // CommandScheduler.getInstance().run();
        driver1.readButtons();
        driver2.readButtons();

        handleDriver1Controls();
        handleDriver2Controls();

        sendTelemetry();
    }

    private void handleDriver1Controls() {
        // --- Controlul motorului de intake ---
        // --- Logica HIBRIDĂ de control pentru Intake ---    // Definim o "zonă moartă" pentru a ignora mișcările accidentale ale joystick-ului
        final double STICK_DEADZONE = 0.1;

        // Citim valoarea joystick-ului
        double joystickPower = -driver1.getLeftY() * 0.9; // Axa Y este inversată

        // 1. Prioritizăm controlul manual de la joystick
        if (Math.abs(joystickPower) > STICK_DEADZONE) {
            // Joystick-ul este mișcat, deci preia controlul.
            intake.setPower(joystickPower);

            // Când folosim joystick-ul, considerăm modul "automat" ca fiind oprit.
            // Astfel, când eliberăm joystick-ul, motorul se va opri (dacă nu era deja în mod automat).
            intakeIsOn = false;
        } else {
            // 2. Joystick-ul este în repaus, deci folosim logica de "toggle" (automat)
            if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                // Inversăm starea modului automat
                intakeIsOn = !intakeIsOn;
            }

            // Setăm puterea pe baza stării modului automat
            if (intakeIsOn) {
                intake.setPower(-0.8); // Viteză constantă în modul automat
            } else {
                intake.stop(); // Oprit
            }
        }


        if (driver1.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            carousel.jogServoPos(CarouselSubsystem1.JOG_ON_POS);
            gamepad1.setLedColor(0, 0, 1, -1);
            double jogPower = Math.abs(driver1.getLeftX()) * -0.25;
            carousel.jogCarousel(jogPower);
            if (driver1.wasJustPressed(GamepadKeys.Button.START)) {
                carousel.confirmAlignment();
                gamepad1.rumble(400);
                gamepad1.setLedColor(0, 1, 0, 1500);
            }
            return;
        }
        else{
            carousel.jogServoPos(CarouselSubsystem1.JOG_OFF_POS);
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
        }

        if (carousel.isReadyToShoot() && !hasRumbled) {
            gamepad1.rumble(500);
            hasRumbled = true;
        }

        if (carousel.getOuttakeState().equals("OUT_IDLE")) outtakePrepared = false;
    }

    private void handleDriver2Controls() {
        if (driver2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) carousel.triggerShoot();

        // --- Tranziții de Stare (Toggle) ---
        if (driver2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            if (turretTeleOpState == TurretTeleOpState.MANUAL) {
                turretTeleOpState = TurretTeleOpState.SEMI_AUTO_SEARCHING;
            } else {
                turretTeleOpState = TurretTeleOpState.MANUAL;
                turret.setManualControl(0); // Forțează ieșirea din orice mod auto al subsistemului
            }
        }

        // --- Acțiuni pe Baza Stării ---
        AprilTagDetection bestTag = vision.getBestDetection();
        boolean hasValidTarget = (bestTag != null && bestTag.metadata != null && (bestTag.id == 20 || bestTag.id == 24));

        switch (turretTeleOpState) {
            case MANUAL:
                driver2.gamepad.setLedColor(0, 1, 0, -1); // LED Verde solid
                turret.setManualControl(driver2.getRightX());
                if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) turret.setTargetAngle(0.0);
                if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) turret.setTargetAngle(-90.0);
                if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) turret.setTargetAngle(90.0);
                if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) turret.goHome();
                break;

            case SEMI_AUTO_SEARCHING:
                driver2.gamepad.setLedColor(1, 0, 0, -1); // LED Roșu
                if (hasValidTarget) {
                    turretTeleOpState = TurretTeleOpState.SEMI_AUTO_LOCKING;
                    driver2.gamepad.rumble(250); // Vibrație scurtă, unică, la detecție
                    lockOnTimer.reset();
                    turret.commandAutoAim(bestTag);
                } else {
                    turret.setManualControl(driver2.getRightX());
                }
                break;

            case SEMI_AUTO_LOCKING:
                driver2.gamepad.setLedColor(1, 0, 0, -1); // LED Roșu
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
    }

    private void sendTelemetry() {
        AprilTagDetection bestTag = vision.getBestDetection();
        double bearing = (bestTag != null && bestTag.ftcPose != null) ? bestTag.ftcPose.bearing : 0.0;

        telemetry.addLine("--- TURELA ---");
        telemetry.addData("TeleOp State", turretTeleOpState);
        telemetry.addData("Subsystem State", turret.getControlState());
        telemetry.addData("Target Angle", "%.1f", turret.getTargetAngle());
        telemetry.addData("Current Angle", "%.1f", turret.getCurrentAngle());
        telemetry.addData("AprilTag Bearing", "%.1f", bearing);
        telemetry.addLine("\n--- CARUSEL ---");
        telemetry.addData("Outtake State", carousel.getOuttakeState());
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("01.Global index", carousel.getGlobalIndex());
        packet.put("02.LogicalIndex", carousel.getLogicalIndex());
        packet.put("03.CarouselTarget Position", carousel.getTargetPosition());
        packet.put("04.Actual Position", carousel.getCurrentPosition());
        packet.put("05.Distance", carousel.getDistance());
        packet.put("06.Occupied 0", carousel.getOccupied(0));
        packet.put("07.Occupied 1", carousel.getOccupied(1));
        packet.put("08.Occupied 2", carousel.getOccupied(2));
        packet.put("09.Hue1", carousel.getHue1());
        packet.put("10.Hue2", carousel.getHue2());
        packet.put("11.HueMax", carousel.getHueMax());
        packet.put("12.Slot colors", carousel.getSlotsColorString());
        packet.put("13.Outtake order string", carousel.getOuttakeOrderString());
        packet.put("14.Turret TeleOp State", turretTeleOpState.name());
        packet.put("15.Turret Subsystem State", turret.getControlState().name());
        packet.put("16.Turret Target", turret.getTargetAngle());
        packet.put("17.Turret Current", turret.getCurrentAngle());
        packet.put("18.AprilTag Bearing", bearing);
        dashboard.sendTelemetryPacket(packet);
    }
}
