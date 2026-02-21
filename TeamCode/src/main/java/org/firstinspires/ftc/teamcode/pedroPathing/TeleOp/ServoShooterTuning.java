package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.util.MathUtils;

@Config
@TeleOp(name = "Servo Shooter Tuning", group = "Util")
public class ServoShooterTuning extends LinearOpMode {

    private Servo servo1, servo2;
    private FtcDashboard dashboard;

    // Acestea sunt variabilele pe care le vom controla din FtcDashboard
    public static double masterPosition = 0.5; // Poziția principală
    public static double servo2Offset = 0.0;   // Offset-ul pe care îl vom regla

    @Override
    public void runOpMode() throws InterruptedException {
        // Inițializare hardware
        servo1 = hardwareMap.get(Servo.class, "angleServo1"); // Numele primului servo
        servo2 = hardwareMap.get(Servo.class, "angleServo2"); // Numele celui de-al doilea servo

        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Gata de test. Deschide FtcDashboard.");
        telemetry.addLine("Mișcă slider-ul 'masterPosition' pentru a vedea cum se mișcă servourile.");
        telemetry.addLine("Ajustează 'servo2Offset' până când zgomotul de 'luptă' dispare.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Asigură-te că poziția master este în limitele valide (0.0 - 1.0)
            masterPosition = MathUtils.clamp(masterPosition, 0.0, 1.0);

            // Calculează poziția pentru al doilea servo, în oglindă și cu offset
            double servo2TargetPos = 1.0 - masterPosition + servo2Offset;
            servo2TargetPos = MathUtils.clamp(servo2TargetPos, 0.0, 1.0);

            // Comandă servourile
            servo1.setPosition(masterPosition);
            servo2.setPosition(servo2TargetPos);

            // Trimite date către FtcDashboard și telemetrie
            sendTelemetry(masterPosition, servo2TargetPos);
        }
    }

    private void sendTelemetry(double pos1, double pos2) {
        telemetry.addData("Master Position (Servo1)", "%.3f", pos1);
        telemetry.addData("Servo2 Offset", "%.4f", servo2Offset);
        telemetry.addData("Servo2 Target (calculat)", "%.3f", pos2);
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Servo1 Position", pos1);
        packet.put("Servo2 Position", pos2);
        packet.put("Servo2 Offset", servo2Offset);
        dashboard.sendTelemetryPacket(packet);
    }
}
