package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem1;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.VisionSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="TeleOp_Final_cu_Turela")
public class TeleOpCarousel1 extends OpMode {

    private GamepadEx driver1;
    private GamepadEx driver2;

    private CarouselSubsystem1 carousel;
    private TurretSubsystem turret;
    private VisionSubsystem vision;

    private FtcDashboard dashboard;

    // Stări pentru carusel
    private boolean outtakePrepared = false;
    private boolean hasRumbled = false;

    // --- NOU: Mașină de stări pentru controlul turelei din TeleOp ---
    private enum TurretTeleOpState { MANUAL, SEMI_AUTO_SEARCHING, SEMI_AUTO_LOCKING }
    private TurretTeleOpState turretTeleOpState = TurretTeleOpState.MANUAL;
    private final ElapsedTime lockOnTimer = new ElapsedTime();

    @Override
    public void init() {
        // --- SOLUȚIA: Asigură o stare curată la fiecare inițializare ---
        CommandScheduler.getInstance().reset();

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
        carousel = new CarouselSubsystem1(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
        carousel.resetForStart();
        carousel.activateIntake();

        dashboard = FtcDashboard.getInstance();
        CommandScheduler.getInstance().registerSubsystem(carousel, turret, vision);

        telemetry.addLine("INIT: gata de START...");
        telemetry.update();
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        driver1.readButtons();
        driver2.readButtons();

        handleCarouselControls();
        handleTurretControls();

        sendTelemetry();
    }

    private void handleCarouselControls() {
        // ... (codul pentru carusel rămâne neschimbat)
    }

    private void handleTurretControls() {
        // --- Tranziții de Stare (Toggle) ---
        if (driver2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
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
                        // Suntem pe țintă. Verificăm de cât timp.
                        if (lockOnTimer.milliseconds() > 500) {
                            // Suntem pe țintă de >500ms, deci vibrăm continuu.
                            driver2.gamepad.rumble(0.7, 0.7, 200);
                        }
                        // Cât timp așteptăm cele 500ms, nu facem nimic (fără vibrații).
                    } else {
                        // Nu suntem pe țintă, dar o vedem. Resetăm cronometrul și nu vibrăm.
                        lockOnTimer.reset();
                    }
                } else {
                    // Am pierdut ținta, revenim la căutare manuală.
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
        packet.put("Turret TeleOp State", turretTeleOpState.name());
        packet.put("Turret Subsystem State", turret.getControlState().name());
        packet.put("Turret Target", turret.getTargetAngle());
        packet.put("Turret Current", turret.getCurrentAngle());
        packet.put("AprilTag Bearing", bearing);
        dashboard.sendTelemetryPacket(packet);
    }
}
