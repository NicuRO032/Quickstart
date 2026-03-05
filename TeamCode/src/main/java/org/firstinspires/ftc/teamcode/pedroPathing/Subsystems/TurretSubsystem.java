package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.acmerobotics.dashboard.config.Config;import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController; // IMPORTANT: Importă PIDController
import com.seattlesolvers.solverslib.util.MathUtils;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@Config
public class TurretSubsystem extends SubsystemBase {

    private final Servo turretServo;
    private final Servo angleServo1;
    private final Servo angleServo2;

    // --- Constante Rotație Turelă ---
    public static double GEAR_RATIO = 2.0;
    public static double SERVO_RANGE_DEGREES = 360.0;
    public static double TURRET_MIN_ANGLE = -55.0;
    public static double TURRET_MAX_ANGLE = 55.0;
    public static double HOME_ANGLE = 0.0;
    public static double MANUAL_SPEED_MULTIPLIER = 1.0;
    public static double AIMING_TOLERANCE_DEGREES = 1.0; // Toleranță mai mică pentru o ochire mai precisă

    // --- Coeficienți pentru noul controler PID de viteză ---
    // Aceste valori sunt un punct de pornire și vor necesita reglaj fin (tuning)
    public static double AIMING_KP = 0.4;  // Răspunsul proporțional la eroare
    public static double AIMING_KI = 0.0;  // Anulează erorile mici, persistente
    public static double AIMING_KD = 0.03;  // Previne oscilațiile și stabilizează mișcarea

    // --- Constante Unghi Shooter ---
    public static double ANGLE_MIN_POS = 0.06;
    public static double ANGLE_MAX_POS = 0.37;
    public static double ANGLE_MANUAL_SENSITIVITY = 0.02;
    public static double ANGLE_HOME_POS = 0.06;
    public static double ANGLE_SERVO2_OFFSET = 0.07;

    // --- Stări și Logică ---
    // Am eliminat SWEEPING_FOR_TAG
    public enum ControlState { MANUAL_CONTROL, HOLDING_POSITION, TRACKING_TAG }
    private ControlState currentState = ControlState.HOLDING_POSITION;

    private enum AngleControlState { MANUAL, PROGRAMMATIC }
    private AngleControlState angleControlState = AngleControlState.PROGRAMMATIC;

    private final PIDController turretPID; // Noul obiect PID Controller
    private double programTargetAngle = HOME_ANGLE;
    private double manualServoPosition;
    public static double currentShooterAnglePos;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        angleServo1 = hardwareMap.get(Servo.class, "angleServo1");
        angleServo2 = hardwareMap.get(Servo.class, "angleServo2");

        // Inițializăm noul controler PID
        turretPID = new PIDController(AIMING_KP, AIMING_KI, AIMING_KD);
        turretPID.setSetPoint(0.0); // Ținta PID-ului este să aducă eroarea (bearing-ul) la ZERO

        goHome();
        setShooterAngle(ANGLE_HOME_POS);
    }

    // --- Metode de Control Turelă (Logică nouă) ---

    /**
     * Metoda principală pentru auto-aim, bazată pe controlul vitezei.
     * @param bestTag Cel mai bun AprilTag detectat.
     */
    /**
     * Metoda pentru auto-aim care primește date direct de la Limelight (via VisionSubsystem)
     * @param targetVisible Dacă Limelight vede un tag valid
     * @param seenTagId ID-ul tag-ului detectat curent
     * @param bearingError Eroarea orizontală (tx de la Limelight)
     * @param targetId ID-ul pe care robotul trebuie să îl urmărească (ex: 20 sau 24)
     */
    public void commandAutoAim(boolean targetVisible, int seenTagId, double bearingError, int targetId) {
        // Verificăm dacă ținta este vizibilă ȘI are ID-ul corect pentru alianță
        if (targetVisible && seenTagId == targetId) {
            currentState = ControlState.TRACKING_TAG;

            // Dacă eroarea este mai mică decât toleranța, nu mai actualizăm programTargetAngle
            // pentru a preveni tremuratul (jitter)
            if (Math.abs(bearingError) < AIMING_TOLERANCE_DEGREES) {
                return;
            }

            // Deoarece ai un SERVO DE POZIȚIE, PID-ul calculează o "corecție"
            // care se adaugă la unghiul actual.
            // Setpoint este 0.0 (vrem tag-ul pe centru), măsura este bearingError.
            double correction = turretPID.calculate(bearingError, 0.0);

            // Actualizăm unghiul țintă al servoului
            programTargetAngle = getCurrentAngle() + correction;

            // Asigurăm limitarea unghiului în limitele fizice [-55, 55]
            programTargetAngle = MathUtils.clamp(programTargetAngle, TURRET_MIN_ANGLE, TURRET_MAX_ANGLE);

        } else {
            // Dacă am pierdut ținta, trecem în HOLDING pe ultima poziție cunoscută
            if (currentState == ControlState.TRACKING_TAG) {
                currentState = ControlState.HOLDING_POSITION;
                programTargetAngle = getCurrentAngle();
            }
        }
    }

    public void setTargetAngle(double angle) {
        this.programTargetAngle = MathUtils.clamp(angle, TURRET_MIN_ANGLE, TURRET_MAX_ANGLE);
        this.currentState = ControlState.HOLDING_POSITION;
    }

    public void setManualControl(double input) {
        final double STICK_DEADZONE = 0.05;
        if (Math.abs(input) > STICK_DEADZONE) {
            if (currentState != ControlState.MANUAL_CONTROL) {
                manualServoPosition = turretServo.getPosition();
            }
            this.currentState = ControlState.MANUAL_CONTROL;
            double positionChangePerLoop = (input * MANUAL_SPEED_MULTIPLIER) / 50.0;
            manualServoPosition += positionChangePerLoop;
            turretServo.setPosition(MathUtils.clamp(manualServoPosition, 0.0, 1.0));
        } else {
            // La eliberarea joystick-ului, dacă eram în manual sau tracking, trecem în HOLDING
            if (currentState == ControlState.MANUAL_CONTROL || currentState == ControlState.TRACKING_TAG) {
                this.programTargetAngle = getCurrentAngle();
                this.currentState = ControlState.HOLDING_POSITION;
            }
        }
    }

    public void goHome() {
        setTargetAngle(HOME_ANGLE);
    }

    // --- Metode Unghi Shooter (Neschimbate) ---
    public void setShooterAngle(double position) {
        this.angleControlState = AngleControlState.PROGRAMMATIC;
        this.currentShooterAnglePos = MathUtils.clamp(position, ANGLE_MIN_POS, ANGLE_MAX_POS);
    }

    public void setManualShooterAngle(double input) {
        final double STICK_DEADZONE = 0.1;
        if (Math.abs(input) > STICK_DEADZONE) {
            this.angleControlState = AngleControlState.MANUAL;
            double positionChange = -input * ANGLE_MANUAL_SENSITIVITY;
            this.currentShooterAnglePos += positionChange;
            this.currentShooterAnglePos = MathUtils.clamp(currentShooterAnglePos, ANGLE_MIN_POS, ANGLE_MAX_POS);
        }
    }

    public double getCurrentShooterAngle(){
        return currentShooterAnglePos;
    }

    @Override
    public void periodic() {
        // Logica din periodic devine mai simplă.
        turretPID.setPID(AIMING_KP, AIMING_KI, AIMING_KD);
        if (currentState != ControlState.MANUAL_CONTROL) {
            // Asigură clamarea unghiului înainte de a-l trimite la servo
            double clampedAngle = MathUtils.clamp(programTargetAngle, TURRET_MIN_ANGLE, TURRET_MAX_ANGLE);
            turretServo.setPosition(turretAngleToServoPosition(clampedAngle));
        }

        // Update unghi shooter (neschimbat)
        angleServo1.setPosition(currentShooterAnglePos);
        double servo2Pos = 1.0 - currentShooterAnglePos + ANGLE_SERVO2_OFFSET;
        angleServo2.setPosition(MathUtils.clamp(servo2Pos, 0.0, 1.0));
    }

    // --- Convertoare și Gettere (Neschimbate) ---
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

    public ControlState getControlState() { return currentState; }
    public double getTargetAngle() { return programTargetAngle; }
    public double getShooterAnglePosition() { return currentShooterAnglePos; }
    public String getAngleControlState() { return angleControlState.toString(); }
}
