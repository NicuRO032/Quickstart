package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

//git
@TeleOp(name = "ConceptScanServo", group = "TeleOp")
@Config
public class ConceptScanServo extends LinearOpMode {

    private Servo servo1, servo2;
    private DcMotorEx shooterMotor1, shooterMotor2;
    private AnalogInput analogFeedback;
    private FtcDashboard dashboard;

    // --- Variabile pentru Carusel Servo Tuning ---
    public static double servoPos = 0.224;      // Poziția master pentru servo1
    public static double servo2Offset = 0.02;    // NOU: Offset pentru servo2

    // --- Constante și Variabile pentru Shooter (neschimbate) ---
    public static double SHOOTER_kP = 0.003;
    public static double SHOOTER_kI = 0.0;
    public static double SHOOTER_kD = 0.00001;
    public static double SHOOTER_kF = 0.00046; // Coeficient Feedforward
    public static double TARGET_SHOOTER_RPM = 3000.0; // Ținta pentru shooter, în RPM
    public static final double SHOOTER_MOTOR_CPR = 28.0; // Counts Per Revolution pentru motorul de shooter

    private PIDController shooterController; // Folosim PIDController-ul original

    // --- Funcții ajutătoare pentru conversie RPM <-> Ticks/Secundă ---
    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * SHOOTER_MOTOR_CPR) / 60.0;
    }

    private double ticksPerSecondToRpm(double ticksPerSecond) {
        return (ticksPerSecond * 60.0) / SHOOTER_MOTOR_CPR;
    }


    @Override
    public void runOpMode() throws InterruptedException {

        // --- Inițializare Hardware ---
        servo1 = hardwareMap.get(Servo.class, "carouselServo1");
        servo2 = hardwareMap.get(Servo.class, "carouselServo2");

        analogFeedback = hardwareMap.get(AnalogInput.class, "axonFeedback");
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "motorShooter1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "motorShooter2");

        // --- Configurare Shooter ---
        shooterMotor1.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor2.setDirection(DcMotorEx.Direction.FORWARD);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterController = new PIDController(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD); // Inițializare PID simplu

        // --- Inițializare Dashboard și poziție inițială ---
        dashboard = FtcDashboard.getInstance();
        servo1.setPosition(servoPos);
        servo2.setPosition(servoPos); // Poziția inițială este aceeași

        telemetry.addData("Status", "Initializat. Astept START.");
        telemetry.update();
        //shooterMotor1.setCurrentAlert(5, CurrentUnit.AMPS);
        //shooterMotor2.setCurrentAlert(5, CurrentUnit.AMPS);

        waitForStart();

        while (opModeIsActive()) {

            // ================== LOGICA PENTRU SHOOTER (PID + F - Originală) ==================
            // (Această secțiune este exact ca în fișierul tău inițial)
            shooterController.setPID(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD);

            double currentShooterVelo = shooterMotor1.getVelocity();
            double targetShooterVelo = rpmToTicksPerSecond(TARGET_SHOOTER_RPM);

            double pidCorrection = shooterController.calculate(currentShooterVelo, targetShooterVelo);
            double feedforward = targetShooterVelo * SHOOTER_kF;

            double shooterPower = feedforward + pidCorrection;
            shooterMotor1.setPower(shooterPower);
            shooterMotor2.setPower(shooterPower);


            // ================== LOGICA PENTRU SERVO (Carusel - cu Tuning Corect) ==================
            if (gamepad1.y) {
                servoPos += 0.002;
                sleep(50);
            }
            if (gamepad1.x) {
                servoPos -= 0.002;
                sleep(50);
            }
            if (gamepad1.a) {
                servoPos = 0.5;
            }

            // MODIFICARE: Logica de comandă a servourilor pentru a include offset-ul
            servoPos = MathUtils.clamp(servoPos, 0.0, 1.0); // Limitează poziția master

            // Comandă servo1 direct
            servo1.setPosition(servoPos);

            // Calculează și comandă servo2 ca fiind egal cu servo1, plus offset
            double servo2TargetPos = servoPos + servo2Offset; // <-- AICI ESTE MODIFICAREA
            servo2.setPosition(MathUtils.clamp(servo2TargetPos, 0.0, 1.0));

            double currentMotor1 = shooterMotor1.getCurrent(CurrentUnit.AMPS);
            double currentMotor2 = shooterMotor1.getCurrent(CurrentUnit.AMPS);


            // ================== TELEMETRIE ==================
            double feedbackVoltage = analogFeedback.getVoltage();
            double currentShooterRPM = ticksPerSecondToRpm(currentShooterVelo);

            // Telemetrie pe telefon
            telemetry.addData("Servo1 Position (Master)", "%.3f", servoPos);
            telemetry.addData("Servo2 Offset", "%.4f", servo2Offset);
            telemetry.addData("Servo2 Calculated Target", "%.3f", servo2TargetPos);
            telemetry.addData("Feedback Voltage", "%.3f V", feedbackVoltage);
            telemetry.addData("Target RPM", "%.1f", TARGET_SHOOTER_RPM);
            telemetry.addData("Current RPM", "%.1f", currentShooterRPM);
            telemetry.update();

            // Telemetrie pe FtcDashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Servo Commanded Position", servoPos);
            packet.put("Servo Feedback Voltage (mV)", feedbackVoltage * 1000);
            packet.put("Shooter Target RPM", TARGET_SHOOTER_RPM);
            packet.put("Shooter Current RPM", currentShooterRPM);
            packet.put("Shooter Power", shooterPower);
            packet.put("Shooter Error (RPM)", TARGET_SHOOTER_RPM - currentShooterRPM);
            // Adăugăm și datele de tuning pentru servo
            packet.put("Servo2 Offset", servo2Offset);
            packet.put("Servo2 Target", servo2TargetPos);

            packet.put("Motor 1 Amps", String.format("%.3f", currentMotor1));
            packet.put("Motor 2 Amps", String.format("%.3f", currentMotor2));
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
