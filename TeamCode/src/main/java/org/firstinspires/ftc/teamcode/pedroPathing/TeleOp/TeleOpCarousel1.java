package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem1;

@TeleOp(name="TeleOpCarousel_Final_Robust")
public class TeleOpCarousel1 extends OpMode {

    private GamepadEx driver1;
    private CarouselSubsystem1 carousel;
    private FtcDashboard dashboard;

    private CarouselSubsystem1.OuttakePattern selectedPattern = CarouselSubsystem1.OuttakePattern.PGG;
    private boolean outtakePrepared = false;
    private boolean hasRumbled = false;

    @Override
    public void init() {
        driver1 = new GamepadEx(gamepad1);
        carousel = new CarouselSubsystem1(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        CommandScheduler.getInstance().registerSubsystem(carousel);
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        driver1.readButtons();

        // 1. URGENȚĂ & MANUAL
        if (driver1.wasJustPressed(GamepadKeys.Button.BACK)) {
            carousel.abortAll();
            outtakePrepared = false;
            hasRumbled = false;
            gamepad1.setLedColor(1, 0, 0, 1000);
        }
        if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) carousel.manualStepLeft();
        if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) carousel.manualStepRight();

        // 2. PATTERN (X, Y, B)
        if (driver1.wasJustPressed(GamepadKeys.Button.X)) selectedPattern = CarouselSubsystem1.OuttakePattern.PGG;
        if (driver1.wasJustPressed(GamepadKeys.Button.Y)) selectedPattern = CarouselSubsystem1.OuttakePattern.GPG;
        if (driver1.wasJustPressed(GamepadKeys.Button.B)) selectedPattern = CarouselSubsystem1.OuttakePattern.GGP;

        // 3. PREGĂTIRE
        if ((carousel.allSlotsOccupied() || driver1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
                && !outtakePrepared && carousel.getOuttakeState().equals("OUT_IDLE")) {
            carousel.prepareOuttake(selectedPattern);
            outtakePrepared = true;
            hasRumbled = false;
        }

        // VIBRAȚIE READY
        if (carousel.isReadyToShoot() && !hasRumbled) {
            gamepad1.rumble(500);
            gamepad1.setLedColor(1, 1, 1, 2000);
            hasRumbled = true;
        }

        // 4. TRĂGACI (RIGHT BUMPER)
        if (driver1.wasJustPressed(GamepadKeys.Button.A)) {
            carousel.triggerShoot();
        }

        // RESET FLAG
        if (carousel.getOuttakeState().equals("OUT_IDLE")) outtakePrepared = false;

        sendDashboardData();
    }

    private void sendDashboardData() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("IntakeState", carousel.getIntakeState());
        packet.put("OuttakeState", carousel.getOuttakeState());
        packet.put("Logical Index", carousel.getLogicalIndex());
        packet.put("Global Index", carousel.getGlobalIndex());
        packet.put("Order", carousel.getOuttakeOrderString());
        packet.put("Ptr", carousel.getOuttakePtr());
        packet.put("Target", carousel.getTargetPosition());
        packet.put("Actual", carousel.getCurrentPosition());

        // Starea ocupării (True/False)
        packet.put("Slots Occupied", String.format("[%b, %b, %b]",
                carousel.getOccupied(0), carousel.getOccupied(1), carousel.getOccupied(2)));

        // Culorile detectate (Green/Purple/Empty)
        packet.put("Slots Colors", carousel.getSlotsColorString());


        dashboard.sendTelemetryPacket(packet);
    }
}