package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "PIDF Position Dashboard Tuner", group = "Testing")
@Config  // 🔥 permite reglaj live din Dashboard
public class PIDF_PositionDashboardTuner extends LinearOpMode {

    // 🔧 Coeficienți reglabil prin FTC Dashboard
    public static double P = 15.0;
    public static double kP = 15.0;
    public static double kI = 0;
    public static double kD = 5.0;
    public static double kF = 0;

    public static double pow = 1.0;
    //public static int tolerance = 10;

    public static long refresh = 50; //miliseconds

    // 🎯 Țintă reglabilă (ticks)
    public int index = 0;
    public int carouselPosition = 0;
    public float pas = 128.1666666f;
    public int targetPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // 1️⃣ Inițializează motorul (numele trebuie să corespundă cu cel din config)
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motorCarusel");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocityPIDFCoefficients(kP,kI,kD,kF);


        //motor.setTargetPositionTolerance(tolerance);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPositionPIDFCoefficients(P);
        //motor.setTargetPositionTolerance(tolerance);





        // 2️⃣ Inițializează Dashboard-ul
        FtcDashboard dashboard = FtcDashboard.getInstance();

        PIDFCoefficients pidfOrig = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
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
            //PIDFCoefficients pidf = new PIDFCoefficients(kP, kI, kD, kF);
            //motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidf);
            //motor.setPositionPIDFCoefficients(kP);

            // 4️⃣ Setează pozitia dorită
            //motor.setTargetPositionTolerance(tolerance);


            if(gamepad1.dpadUpWasPressed()){
                index = index + 1;
                carouselPosition = Math.round(index * pas);
                targetPosition = carouselPosition;
            }
            if(gamepad1.dpadDownWasPressed()){
                index = index - 1;
                carouselPosition = Math.round(index * pas);
                targetPosition = carouselPosition;
            }

            motor.setTargetPosition(targetPosition);
            motor.setPower(pow);





            // 5️⃣ Obține pozitia măsurată
            int actualPosition = motor.getCurrentPosition();

            // 6️⃣ Trimite datele la Dashboard (pentru grafic)
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Index", index);
            packet.put("Target Position", targetPosition);
            packet.put("Actual Position", actualPosition);
            packet.put("Actual Power", motor.getPower());
            dashboard.sendTelemetryPacket(packet);

            /**
            // 7️⃣ Trimite date și în telemetria normală (Driver Station)
            telemetry.addData("Target Position", "%.1f", targetPosition);
            telemetry.addData("Actual Position", "%.1f", actualPosition);
            telemetry.addLine(String.format("P: %.2f  I: %.2f  D: %.2f  F: %.2f", kP, kI, kD, kF));
            **/


            sleep(refresh);  // update rate
        }
    }
}
