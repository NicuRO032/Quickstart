package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.MathUtils;

@Config
public class TurretSubsystem extends SubsystemBase {

    // --- HARDWARE ---
    private final Servo turretServo;

    // Raportul de transmisie mecanică (Gear Ratio)
    // Formula: (Număr dinți roată mare de pe turelă) / (Număr dinți pinion de pe servo)
    public static final double GEAR_RATIO = 2.0; // Exemplu: O roată de 90 de dinți pe turelă și un pinion de 30 pe servo => 90 / 30 = 3.0

    // Cursa totală a servomotorului în grade (GoBilda 5-Turn: 5 rotații * ~280-300 grade/rotație)
    // Este mai sigur să folosești o valoare puțin mai mică pentru a evita forțarea la capete.
    public static  double SERVO_RANGE_DEGREES = 2100.0; // ~5 rotații * 280 grade

    // Limitele de rotație ale TURELEI în grade. Poți limita cursa pentru a nu lovi alte părți ale robotului.
    public static final double TURRET_MIN_ANGLE = -180.0; // Exemplu: -180 grade (stânga)
    public static final double TURRET_MAX_ANGLE = 180.0;  // Exemplu: +180 grade (dreapta)

    // Unghiul "zero" sau de referință al turelei, în grade, față de robot (0 = drept înainte).
    public static final double HOME_ANGLE = 0.0;

    // Multiplicator pentru viteza de rotație în modul manual. Un număr mai mare = viteză mai mare.
    public static final double MANUAL_SPEED_MULTIPLIER = 1; // VITEZA MAXIMĂ A SERVOULUI (de ex. 1.0 = 100% din cursă/sec)


    // --- STĂRI (STATES) ---
    public enum ControlMode {
        PROGRAM_CONTROL,
        MANUAL_CONTROL
    }

    private ControlMode currentMode = ControlMode.PROGRAM_CONTROL;
    private double programTargetAngle = HOME_ANGLE;
    private double manualServoPosition;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        //goHome();
    }

    public void setTargetAngle(double angle) {
        this.programTargetAngle = MathUtils.clamp(angle, TURRET_MIN_ANGLE, TURRET_MAX_ANGLE);
        this.currentMode = ControlMode.PROGRAM_CONTROL;
    }

    public void setManualControl(double input) {
        final double STICK_DEADZONE = 0.05;

        if (Math.abs(input) > STICK_DEADZONE) {
            // --- A intrat în modul MANUAL ---
            if (currentMode == ControlMode.PROGRAM_CONTROL) {
                // Sincronizăm poziția la prima activare
                manualServoPosition = turretServo.getPosition();
            }
            this.currentMode = ControlMode.MANUAL_CONTROL;

            // --- CORECȚIE VITEZĂ (Pasul 1) ---
            // Calcul direct: viteza maximă este MANUAL_SPEED_MULTIPLIER (procent din cursa totală/secundă)
            // Împărțim la 50 pentru a obține modificarea per ciclu de loop (presupunând ~50Hz).
            double positionChangePerLoop = (input * MANUAL_SPEED_MULTIPLIER) / 50.0;

            manualServoPosition += positionChangePerLoop;
            turretServo.setPosition(MathUtils.clamp(manualServoPosition, 0.0, 1.0));

        } else {
            // --- A ieșit din modul MANUAL ---
            if (currentMode == ControlMode.MANUAL_CONTROL) {
                // --- CORECȚIE PERSISTENȚĂ POZIȚIE (Pasul 2) ---
                // Actualizăm ținta programatică cu unghiul curent la care a ajuns turela.
                this.programTargetAngle = getCurrentAngle();
            }
            this.currentMode = ControlMode.PROGRAM_CONTROL;
        }
    }

    public void goHome() {
        setTargetAngle(HOME_ANGLE);
    }

    @Override
    public void periodic() {
        if (currentMode == ControlMode.PROGRAM_CONTROL) {
            double servoPosition = turretAngleToServoPosition(programTargetAngle);
            turretServo.setPosition(servoPosition);
        }
    }

    private double turretAngleToServoPosition(double turretAngle) {
        double servoAngle = turretAngle * GEAR_RATIO;
        double servoPosition = 0.5 + (servoAngle / SERVO_RANGE_DEGREES);
        return MathUtils.clamp(servoPosition, 0.0, 1.0);
    }

    public double getCurrentAngle() {
        double servoPosition = turretServo.getPosition();
        double servoAngle = (servoPosition - 0.5) * SERVO_RANGE_DEGREES;
        return servoAngle / GEAR_RATIO;
    }

    public ControlMode getControlMode() { return currentMode; }
    public double getTargetAngle() { return programTargetAngle; }
}
