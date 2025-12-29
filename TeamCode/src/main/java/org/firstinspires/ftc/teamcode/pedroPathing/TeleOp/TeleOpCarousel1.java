package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

    private CarouselSubsystem1.OuttakePattern selectedPattern = CarouselSubsystem1.OuttakePattern.PGG;
    private boolean outtakePrepared = false;
    private boolean hasRumbled = false;

    @Override
    public void init() {
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
        carousel = new CarouselSubsystem1(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
        carousel.resetForStart();
        carousel.activateIntake();

        dashboard = FtcDashboard.getInstance();
        CommandScheduler.getInstance().registerSubsystem(carousel);
        CommandScheduler.getInstance().registerSubsystem(turret);
        CommandScheduler.getInstance().registerSubsystem(vision);

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
        if (driver1.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            gamepad1.setLedColor(0, 0, 1, -1);
            double jogPower = driver1.getRightX() * 0.1;
            carousel.jogCarousel(jogPower);
            if (driver1.wasJustPressed(GamepadKeys.Button.START)) {
                carousel.confirmAlignment();
                gamepad1.rumble(400);
                gamepad1.setLedColor(0, 1, 0, 1500);
            }
            return;
        }

        if (driver1.wasJustPressed(GamepadKeys.Button.BACK)) {
            carousel.abortAll();
            outtakePrepared = false;
            hasRumbled = false;
        }

        if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) carousel.manualStepLeft();
        if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) carousel.manualStepRight();

        if (driver1.wasJustPressed(GamepadKeys.Button.X)) carousel.setActivePattern(CarouselSubsystem1.OuttakePattern.PGG);
        if (driver1.wasJustPressed(GamepadKeys.Button.Y)) carousel.setActivePattern(CarouselSubsystem1.OuttakePattern.GPG);
        if (driver1.wasJustPressed(GamepadKeys.Button.B)) carousel.setActivePattern(CarouselSubsystem1.OuttakePattern.GGP);

        if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) && !outtakePrepared) {
            carousel.prepareOuttake(carousel.getActivePattern());
            outtakePrepared = true;
            hasRumbled = false;
        }

        if (carousel.isReadyToShoot() && !hasRumbled) {
            gamepad1.rumble(500);
            hasRumbled = true;
        }

        if (driver1.wasJustPressed(GamepadKeys.Button.A)) carousel.triggerShoot();
        if (carousel.getOuttakeState().equals("OUT_IDLE")) outtakePrepared = false;
    }

    private void handleTurretControls() {
        if (driver2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            AprilTagDetection bestTag = vision.getBestDetection();
            turret.commandAutoAim(bestTag);
        } else {
            turret.setManualControl(driver2.getRightX());

            if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) turret.setTargetAngle(0.0);
            if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) turret.setTargetAngle(-90.0);
            if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) turret.setTargetAngle(90.0);
            if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) turret.goHome();
        }
    }

    private void sendTelemetry() {
        AprilTagDetection bestTag = vision.getBestDetection();
        double bearing = (bestTag != null && bestTag.ftcPose != null) ? bestTag.ftcPose.bearing : 0.0;

        // --- Driver Station Telemetry ---
        telemetry.addLine("--- TURELA ---");
        telemetry.addData("State", turret.getControlState());
        telemetry.addData("Target Angle", "%.1f", turret.getTargetAngle());
        telemetry.addData("Current Angle", "%.1f", turret.getCurrentAngle());
        telemetry.addData("AprilTag Bearing", "%.1f", bearing);

        telemetry.addLine("\n--- CARUSEL ---");
        telemetry.addData("Outtake State", carousel.getOuttakeState());

        telemetry.update();

        // --- FTC Dashboard Telemetry ---
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Turret State", turret.getControlState().name());
        packet.put("Turret Target", turret.getTargetAngle());
        packet.put("Turret Current", turret.getCurrentAngle());
        packet.put("AprilTag Bearing", bearing);
        packet.put("Carousel State", carousel.getOuttakeState());
        dashboard.sendTelemetryPacket(packet);
    }
}
