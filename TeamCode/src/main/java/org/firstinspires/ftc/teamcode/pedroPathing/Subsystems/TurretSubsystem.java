package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.MathUtils;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class TurretSubsystem extends SubsystemBase {

    private final CRServo turretServo; // Modificat în CRServo
    private final Servo angleServo1;
    private final Servo angleServo2;
    private final AnalogInput turretFeedback; // Feedback analogic de la Axon

    // --- Constante Rotație Turelă ---
    public static double GEAR_RATIO = 2.0;
    public static double TURRET_MIN_ANGLE = -55.0;
    public static double TURRET_MAX_ANGLE = 55.0;
    public static double HOME_ANGLE = 0.0;
    public static double MANUAL_POWER_MULTIPLIER = 0.6; // Puterea maximă în mod manual

    // --- Constante Feedback Analogic (Calibrează-le pe acestea!) ---
    // Notează tensiunile citite la limitele fizice folosind ConceptScanServo
    public static double VOLTAGE_AT_MIN = 2.35; // Tensiunea la -55 grade
    public static double VOLTAGE_AT_MAX = 0.75; // Tensiunea la +55 grade

    // --- Coeficienți PID pentru Controlul Vitezei (Auto-Aim) ---
    // În acest mod, PID-ul scoate direct puterea motorului (-1..1)
    public static double AIMING_KP = 0.035;
    public static double AIMING_KI = 0.0;
    public static double AIMING_KD = 0.002;
    public static double AIMING_TOLERANCE_DEGREES = 1.0;

    // --- Constante Unghi Shooter (Neschimbate) ---
    public static double ANGLE_MIN_POS = 0.06;
    public static double ANGLE_MAX_POS = 0.37;
    public static double ANGLE_MANUAL_SENSITIVITY = 0.02;
    public static double ANGLE_HOME_POS = 0.06;
    public static double ANGLE_SERVO2_OFFSET = 0.07;

    // --- Stări și Logică ---
    public enum ControlState { MANUAL_CONTROL, HOLDING_POSITION, TRACKING_TAG }
    private ControlState currentState = ControlState.HOLDING_POSITION;

    private enum AngleControlState { MANUAL, PROGRAMMATIC }
    private AngleControlState angleControlState = AngleControlState.PROGRAMMATIC;

    private final PIDController turretPID;
    private double holdPositionAngle = HOME_ANGLE;
    public static double currentShooterAnglePos;



    public TurretSubsystem(HardwareMap hardwareMap) {
        // Inițializare Hardware
        turretServo = hardwareMap.get(CRServo.class, "turretServo");
        angleServo1 = hardwareMap.get(Servo.class, "angleServo1");
        angleServo2 = hardwareMap.get(Servo.class, "angleServo2");
        turretFeedback = hardwareMap.get(AnalogInput.class, "axonFeedbackTurela"); // Configurat ca Analog Input

        // Inițializare PID
        turretPID = new PIDController(AIMING_KP, AIMING_KI, AIMING_KD);
        turretPID.setSetPoint(0.0); // Țintim eroare (bearing) zero

        setShooterAngle(ANGLE_HOME_POS);
    }

    /**
     * Calculează unghiul curent real al turelei folosind senzorul analogic.
     */
    public double getCurrentAngle() {
        double voltage = turretFeedback.getVoltage();
        // Interpolare liniară pentru a map între tensiune și unghiul fizic
        double normalized = (voltage - VOLTAGE_AT_MIN) / (VOLTAGE_AT_MAX - VOLTAGE_AT_MIN);
        double range = TURRET_MAX_ANGLE - TURRET_MIN_ANGLE;
        return (normalized * range) + TURRET_MIN_ANGLE;
    }

    /**
     * Comandă auto-aim folosind PID pe viteză (putere motor).
     */
    public void commandAutoAim(AprilTagDetection bestTag, int targetId) {
        if (bestTag != null && bestTag.metadata != null && bestTag.id == targetId) {
            currentState = ControlState.TRACKING_TAG;
            double bearingError = bestTag.ftcPose.bearing;

            // VERIFICARE TOLERANȚĂ: Dacă suntem deja aliniați, oprim motorul
            if (Math.abs(bearingError) < AIMING_TOLERANCE_DEGREES) {
                turretServo.setPower(0.0);
                // Opțional: fixăm poziția curentă pentru a nu "aluneca"
                holdPositionAngle = getCurrentAngle();
                return;
            }

            // Dacă nu suntem în toleranță, PID-ul calculează puterea
            // Am folosit semnul care ai zis că merge corect la tine
            double power = turretPID.calculate(bearingError);
            turretServo.setPower(MathUtils.clamp(power, -1.0, 1.0));
        } else {
            // Dacă pierdem tag-ul, oprim turela sau menținem poziția
            if (currentState == ControlState.TRACKING_TAG) {
                setTargetAngle(getCurrentAngle());
            }
            turretServo.setPower(0.0);
        }
    }

    public void setTargetAngle(double angle) {
        this.holdPositionAngle = MathUtils.clamp(angle, TURRET_MIN_ANGLE, TURRET_MAX_ANGLE);
        this.currentState = ControlState.HOLDING_POSITION;
    }

    public void setManualControl(double input) {
        final double STICK_DEADZONE = 0.05;
        if (Math.abs(input) > STICK_DEADZONE) {
            this.currentState = ControlState.MANUAL_CONTROL;
            turretServo.setPower(input * MANUAL_POWER_MULTIPLIER);
        } else if (currentState == ControlState.MANUAL_CONTROL) {
            // Dacă eliberăm joystick-ul, blocăm poziția curentă
            setTargetAngle(getCurrentAngle());
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

    @Override
    public void periodic() {
        turretPID.setPID(AIMING_KP, AIMING_KI, AIMING_KD);

        double currentAngle = getCurrentAngle();
        double currentPower = turretServo.getPower();

        if (currentState == ControlState.HOLDING_POSITION) {
            turretServo.setPower(0);
            /*double error = holdPositionAngle - currentAngle;

            // Dacă eroarea este mai mică de 1 grad (sau cât ai setat), pune puterea 0
            if (Math.abs(error) < AIMING_TOLERANCE_DEGREES) {
                turretServo.setPower(0.0);
            } else {
                double correction = turretPID.calculate(error);
                turretServo.setPower(MathUtils.clamp(correction, -1.0, 1.0));
            }*/
        }

        // --- PROTECȚIA HARDWARE (Soft Stops) ---
        // Păstrează codul existent pentru limite, e foarte bun.
        if (currentAngle <= TURRET_MIN_ANGLE && currentPower < -0.01) {
            turretServo.setPower(0);
        } else if (currentAngle >= TURRET_MAX_ANGLE && currentPower > 0.01) {
            turretServo.setPower(0);
        }

        // 3. Update Unghi Shooter
        angleServo1.setPosition(currentShooterAnglePos);
        double servo2Pos = 1.0 - currentShooterAnglePos + ANGLE_SERVO2_OFFSET;
        angleServo2.setPosition(MathUtils.clamp(servo2Pos, 0.0, 1.0));
    }

    // --- Gettere ---
    public ControlState getControlState() { return currentState; }
    public double getTargetAngle() { return holdPositionAngle; }
    public double getCurrentRealAngle() { return getCurrentAngle(); }
    public double getCurrentTurretFeedbackMv() {
        return turretFeedback.getVoltage();
    }
    public String getAngleControlState() { return angleControlState.toString(); }
    public double getShooterAnglePosition() { return currentShooterAnglePos; }
    public double getCurrentShooterAngle(){return currentShooterAnglePos;
    }
}