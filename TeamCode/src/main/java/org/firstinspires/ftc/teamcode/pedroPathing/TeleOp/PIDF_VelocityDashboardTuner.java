package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "PIDF Velocity Dashboard Tuner", group = "Testing")
@Config  // 🔥 permite reglaj live din Dashboard
public class PIDF_VelocityDashboardTuner extends LinearOpMode {

    // 🔧 Coeficienți reglabil prin FTC Dashboard
    public static double kP = 10.0;
    public static double kI = 3.0;
    public static double kD = 1.0;
    public static double kF = 11.0;

    public static long refresh = 50; //miliseconds

    // 🎯 Țintă reglabilă (ticks/sec)
    public static double targetVelocity = 1000;

    @Override
    public void runOpMode() throws InterruptedException {

        // 1️⃣ Inițializează motorul (numele trebuie să corespundă cu cel din config)
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motorFlywheel");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // 2️⃣ Inițializează Dashboard-ul
        FtcDashboard dashboard = FtcDashboard.getInstance();

        PIDFCoefficients pidfOrig = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("P", pidfOrig.p);
        telemetry.addData("I", pidfOrig.i);
        telemetry.addData("D", pidfOrig.d);
        telemetry.addData("F", pidfOrig.f);
        //telemetry.update();

        telemetry.addLine("Conectează-te la: http://192.168.43.1:8080/dash");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 3️⃣ Aplică coeficienții PIDF curenți
            PIDFCoefficients pidf = new PIDFCoefficients(kP, kI, kD, kF);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

            // 4️⃣ Setează viteza dorită
            motor.setVelocity(targetVelocity);

            // 5️⃣ Obține viteza măsurată
            double actualVelocity = motor.getVelocity();

            // 6️⃣ Trimite datele la Dashboard (pentru grafic)
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target Velocity", targetVelocity);
            packet.put("Actual Velocity", actualVelocity);
            packet.put("Actual Power", motor.getPower());
            dashboard.sendTelemetryPacket(packet);

            // 7️⃣ Trimite date și în telemetria normală (Driver Station)
            telemetry.addData("Target Velocity", "%.1f", targetVelocity);
            telemetry.addData("Actual Velocity", "%.1f", actualVelocity);
            telemetry.addLine(String.format("P: %.2f  I: %.2f  D: %.2f  F: %.2f", kP, kI, kD, kF));
            telemetry.update();

            sleep(refresh);  // update rate
        }
    }
}
