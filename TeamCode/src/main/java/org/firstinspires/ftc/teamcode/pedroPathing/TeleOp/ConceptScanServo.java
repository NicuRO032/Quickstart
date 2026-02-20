

package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.seattlesolvers.solverslib.controller.PIDController;

@TeleOp(name = "ConceptScanServo", group = "TeleOp")
@Config
public class ConceptScanServo extends LinearOpMode {

    private Servo servo1, servo2;
    private DcMotorEx shooterMotor1, shooterMotor2;
    private AnalogInput analogFeedback;
    private FtcDashboard dashboard;

    // --- Variabile pentru Carusel ---
    public static double servoPos = 0.5;   // Poziția comandată pentru servo

    // --- Constante și Variabile pentru Shooter ---
    public static double SHOOTER_kP = 0.003;
    public static double SHOOTER_kI = 0.0;
    public static double SHOOTER_kD = 0.00001;
    public static double SHOOTER_kF = 0.00046; // Coeficient Feedforward
    public static double TARGET_SHOOTER_RPM = 3000.0; // Ținta pentru shooter, în RPM
    public static final double SHOOTER_MOTOR_CPR = 28.0; // Counts Per Revolution pentru motorul de shooter

    private PIDController shooterController;

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

        shooterController = new PIDController(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD);

        // --- Inițializare Dashboard și poziție inițială ---
        dashboard = FtcDashboard.getInstance();
        servo1.setPosition(servoPos);
        servo2.setPosition(servoPos);

        telemetry.addData("Status", "Initializat. Astept START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ================== LOGICA PENTRU SHOOTER (PIDF) ==================
            // Actualizăm coeficienții PID din Dashboard
            shooterController.setPID(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD);

            // Citim viteza curentă a motorului în tick-uri/secundă
            double currentShooterVelo = shooterMotor1.getVelocity();
            // Convertim turația țintă din RPM în tick-uri/secundă
            double targetShooterVelo = rpmToTicksPerSecond(TARGET_SHOOTER_RPM);

            // Calculăm corecția PID
            double pidCorrection = shooterController.calculate(currentShooterVelo, targetShooterVelo);
            // Calculăm termenul Feedforward (proporțional cu viteza țintă)
            double feedforward = targetShooterVelo * SHOOTER_kF;

            // Puterea finală este suma dintre Feedforward și corecția PID
            double shooterPower = feedforward + pidCorrection;
            shooterMotor1.setPower(shooterPower);
            shooterMotor2.setPower(shooterPower);


            // ================== LOGICA PENTRU SERVO (Carusel) ==================
            // Creștere poziție: buton Y
            if (gamepad1.y) {
                servoPos += 0.002;
                sleep(50);  // O mică pauză pentru a permite reglaj fin
            }

            // Scădere poziție: buton X
            if (gamepad1.x) {
                servoPos -= 0.002;
                sleep(50);
            }

            // Buton de resetare rapidă
            if (gamepad1.a) {
                servoPos = 0.5;
            }

            // Limităm poziția servomotorului între 0 și 1
            servoPos = Math.max(0, Math.min(1, servoPos));
            servo1.setPosition(servoPos);
            servo2.setPosition(servoPos);


            // ================== TELEMETRIE ==================
            double feedbackVoltage = analogFeedback.getVoltage();
            double currentShooterRPM = ticksPerSecondToRpm(currentShooterVelo);

            // Telemetrie pe telefon
            telemetry.addData("Servo Position", "%.3f", servoPos);
            telemetry.addData("Feedback Voltage", "%.3f V", feedbackVoltage);
            telemetry.addData("Target RPM", "%.1f", TARGET_SHOOTER_RPM);
            telemetry.addData("Current RPM", "%.1f", currentShooterRPM);
            telemetry.update();

            // Telemetrie pe FtcDashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Servo Commanded Position", servoPos);
            packet.put("Servo Feedback Voltage (mV)", feedbackVoltage * 1000); // Trimitem în milivolți
            packet.put("Shooter Target RPM", TARGET_SHOOTER_RPM);
            packet.put("Shooter Current RPM", currentShooterRPM);
            packet.put("Shooter Power", shooterPower);
            packet.put("Shooter Error (RPM)", TARGET_SHOOTER_RPM - currentShooterRPM);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
