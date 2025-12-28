package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
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
        carousel.resetForStart();
        carousel.activateIntake();

        dashboard = FtcDashboard.getInstance();
        CommandScheduler.getInstance().registerSubsystem(carousel);
        telemetry.addLine("INIT: gata de START...");
        telemetry.update();
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        driver1.readButtons();

        // MOD DE REALINIERE MANUALĂ: Ține apăsat LEFT_BUMPER pentru a activa
        if (driver1.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            // Suprascrie toate celelalte controale

            // Feedback vizual: LED-ul devine albastru
            gamepad1.setLedColor(0, 0, 1, -1);

            // Folosim right_stick_x pentru control analogic fin
            // Scalăm puterea pentru a nu fi prea bruscă
            double jogPower = driver1.getRightX() * 0.1;
            carousel.jogCarousel(jogPower);

            // Butonul de confirmare: setează noul zero (Apăsați START în timp ce țineți LEFT_BUMPER)
            if (driver1.wasJustPressed(GamepadKeys.Button.START)) {
                carousel.confirmAlignment();
                // Feedback haptic și vizual pentru confirmare
                gamepad1.rumble(400);
                gamepad1.setLedColor(0, 1, 0, 1500); // Verde pentru 1.5s
            }

            sendDashboardRealignTelemetry();
            sendDriverStationRealignTelemetry();

            // Ieșim devreme pentru a nu procesa logica normală a butoanelor în acest mod
            return;
        }


        // LOGICA NORMALĂ TELEOP

        // 1. URGENȚĂ & MANUAL
        if (driver1.wasJustPressed(GamepadKeys.Button.BACK)) {
            carousel.abortAll();
            outtakePrepared = false;
            hasRumbled = false;
            gamepad1.setLedColor(1, 0, 0, 1000);
        }
        // Pasul manual este acum pe butoanele mari, D-Pad-ul este liber
        if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) carousel.manualStepLeft();
        if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) carousel.manualStepRight();

        // 2. PATTERN (X, Y, B)
        if (driver1.wasJustPressed(GamepadKeys.Button.X)) {
            selectedPattern = CarouselSubsystem1.OuttakePattern.PGG;
            carousel.setActivePattern(selectedPattern);
        }
        if (driver1.wasJustPressed(GamepadKeys.Button.Y)) {
            selectedPattern = CarouselSubsystem1.OuttakePattern.GPG;
            carousel.setActivePattern(selectedPattern);
        }
        if (driver1.wasJustPressed(GamepadKeys.Button.B)) {
            selectedPattern = CarouselSubsystem1.OuttakePattern.GGP;
            carousel.setActivePattern(selectedPattern);
        }


        // 3. PREGĂTIRE MANUALĂ (DPAD_DOWN) cand vreu sa arunc 1 sau 2 bile, sa nu astept dupa 3
        if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)
                && !outtakePrepared && carousel.getOuttakeState().equals("OUT_IDLE")) {
            carousel.prepareOuttake(selectedPattern);
            outtakePrepared = true;
            hasRumbled = false;
        }

        // Sincronizare flag pentru vibrație și stare
        if (carousel.getOuttakeState().equals("PREPARE_READY")) {
            outtakePrepared = true;
        } else if (carousel.getOuttakeState().equals("OUT_IDLE")) {
            outtakePrepared = false;
        }


        // VIBRAȚIE READY
        if (carousel.isReadyToShoot() && !hasRumbled) {
            gamepad1.rumble(500);
            gamepad1.setLedColor(1, 1, 1, 2000);
            hasRumbled = true;
        }

        // 4. TRĂGACI (A)
        if (driver1.wasJustPressed(GamepadKeys.Button.A)) {
            carousel.triggerShoot();
        }

        // RESET FLAG
        if (carousel.getOuttakeState().equals("OUT_IDLE")) outtakePrepared = false;



        sendDashboardTelemetry();
        sendDriverStationTelemetry();
    }

    /// ////// TELEMETRIE /////////
    private void sendDashboardTelemetry() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.addLine("00.Rulare TeleOP ...");
        packet.put("01.IntakeState", carousel.getIntakeState());
        packet.put("02.OuttakeState", carousel.getOuttakeState());
        packet.put("03.Logical Index", carousel.getLogicalIndex());
        packet.put("04.Global Index", carousel.getGlobalIndex());
        packet.put("05.Target pose", carousel.getTargetPosition());
        packet.put("06.Actual pose", carousel.getCurrentPosition());
        packet.put("07.Slots Occupied", String.format("[%b, %b, %b]",
                carousel.getOccupied(0), carousel.getOccupied(1), carousel.getOccupied(2)));
        packet.put("08.Slots Colors", carousel.getSlotsColorString());
        packet.put("09.Order", carousel.getOuttakeOrderString());
        packet.put("10.Ptr", carousel.getOuttakePtr());

        dashboard.sendTelemetryPacket(packet);
    }

    private void sendDriverStationTelemetry(){
        telemetry.addLine("00.Rulare TeleOP ...");
        telemetry.addData("01.IntakeState", carousel.getIntakeState());
        telemetry.addData("02.OuttakeState", carousel.getOuttakeState());
        telemetry.addData("03.Logical Index", carousel.getLogicalIndex());
        telemetry.addData("04.Global Index", carousel.getGlobalIndex());
        telemetry.addData("05.Target pose", carousel.getTargetPosition());
        telemetry.addData("06.Actual pose", carousel.getCurrentPosition());
        telemetry.addData("07.Slots Occupied", String.format("[%b, %b, %b]",
                carousel.getOccupied(0), carousel.getOccupied(1), carousel.getOccupied(2)));
        telemetry.addData("08.Slots Colors", carousel.getSlotsColorString());
        telemetry.addData("09.Order", carousel.getOuttakeOrderString());
        telemetry.addData("10.Ptr", carousel.getOuttakePtr());
        telemetry.update();
    }

    private void sendDashboardRealignTelemetry() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.addLine("Manual realign SLOT 0 ...");
        packet.put("01.Logical Index", carousel.getLogicalIndex());
        packet.put("02.Global Index", carousel.getGlobalIndex());
        packet.put("03.Target", carousel.getTargetPosition());
        packet.put("04.ACTUAL POSE", carousel.getCurrentPosition());
        dashboard.sendTelemetryPacket(packet);
    }

    private void sendDriverStationRealignTelemetry(){
        telemetry.addLine("Manual realign SLOT 0 ...");
        telemetry.addData("01.Logical Index", carousel.getLogicalIndex());
        telemetry.addData("02.Global Index", carousel.getGlobalIndex());
        telemetry.addData("03.Target", carousel.getTargetPosition());
        telemetry.addData("04.ACTUAL POSE", carousel.getCurrentPosition());
        telemetry.update();
    }

}
