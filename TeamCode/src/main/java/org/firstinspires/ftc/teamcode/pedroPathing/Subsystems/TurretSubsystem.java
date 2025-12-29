package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.MathUtils;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class TurretSubsystem extends SubsystemBase {

    private final Servo turretServo;

    public static double GEAR_RATIO = 2.0;
    public static double SERVO_RANGE_DEGREES = 2100.0;
    public static double TURRET_MIN_ANGLE = -180.0;
    public static double TURRET_MAX_ANGLE = 180.0;
    public static double HOME_ANGLE = 0.0;
    public static double MANUAL_SPEED_MULTIPLIER = 1.0;
    public static double AIMING_P_GAIN = 0.08; // Valoare mai mică pentru stabilitate

    public static double SWEEP_SPEED_DEG_PER_SEC = 30.0; // Viteză redusă pentru a preveni oscilația
    public static double SWEEP_ENDPOINT_1 = -45.0;
    public static double SWEEP_ENDPOINT_2 = 45.0;

    public enum ControlState {
        MANUAL_CONTROL,
        HOLDING_POSITION,
        SWEEPING_FOR_TAG,
        TRACKING_TAG
    }

    private ControlState currentState = ControlState.HOLDING_POSITION;
    private double programTargetAngle = HOME_ANGLE;
    private double manualServoPosition;
    private double sweepDirection = 1.0;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        goHome();
    }

    public void commandAutoAim(AprilTagDetection bestTag) {
        if (bestTag != null && bestTag.metadata != null && (bestTag.id == 20 || bestTag.id == 24)) {
            currentState = ControlState.TRACKING_TAG;
            double bearingError = bestTag.ftcPose.bearing;
            programTargetAngle = getCurrentAngle() + (bearingError * AIMING_P_GAIN);
        } else {
            // Intrăm în baleiere doar dacă nu suntem deja într-un mod automat.
            if (currentState != ControlState.SWEEPING_FOR_TAG && currentState != ControlState.TRACKING_TAG) {
                currentState = ControlState.SWEEPING_FOR_TAG;
                sweepDirection = (getCurrentAngle() < 0) ? 1.0 : -1.0;
            }
        }
    }

    public void setTargetAngle(double angle) {
        this.programTargetAngle = MathUtils.clamp(angle, TURRET_MIN_ANGLE, TURRET_MAX_ANGLE);
        this.currentState = ControlState.HOLDING_POSITION;
    }

    /**
     * Logica de control a fost refactorizată pentru a rezolva eroarea de blocare a stării.
     */
    public void setManualControl(double input) {
        final double STICK_DEADZONE = 0.05;

        if (Math.abs(input) > STICK_DEADZONE) {
            // Când stick-ul este mișcat, intrăm forțat în modul manual.
            if (currentState != ControlState.MANUAL_CONTROL) {
                manualServoPosition = turretServo.getPosition();
            }
            this.currentState = ControlState.MANUAL_CONTROL;

            double positionChangePerLoop = (input * MANUAL_SPEED_MULTIPLIER) / 50.0;
            manualServoPosition += positionChangePerLoop;
            turretServo.setPosition(MathUtils.clamp(manualServoPosition, 0.0, 1.0));

        } else {
            // Când stick-ul NU este mișcat (sau când butonul de auto-aim a fost eliberat),
            // comutăm orice stare activă înapoi la HOLDING_POSITION.
            if (currentState == ControlState.MANUAL_CONTROL || currentState == ControlState.SWEEPING_FOR_TAG || currentState == ControlState.TRACKING_TAG) {
                this.programTargetAngle = getCurrentAngle(); // Salvăm ultimul unghi
                this.currentState = ControlState.HOLDING_POSITION;
            }
        }
    }

    public void goHome() {
        setTargetAngle(HOME_ANGLE);
    }

    @Override
    public void periodic() {
        switch (currentState) {
            case MANUAL_CONTROL:
                // Nu facem nimic, `setManualControl` gestionează totul
                break;

            case HOLDING_POSITION:
            case TRACKING_TAG:
                turretServo.setPosition(turretAngleToServoPosition(programTargetAngle));
                break;

            case SWEEPING_FOR_TAG:
                double increment = (sweepDirection * SWEEP_SPEED_DEG_PER_SEC) / 50.0;
                programTargetAngle += increment;

                if (sweepDirection > 0 && programTargetAngle > SWEEP_ENDPOINT_2) {
                    programTargetAngle = SWEEP_ENDPOINT_2;
                    sweepDirection = -1.0;
                } else if (sweepDirection < 0 && programTargetAngle < SWEEP_ENDPOINT_1) {
                    programTargetAngle = SWEEP_ENDPOINT_1;
                    sweepDirection = 1.0;
                }

                turretServo.setPosition(turretAngleToServoPosition(programTargetAngle));
                break;
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

    public ControlState getControlState() { return currentState; }
    public double getTargetAngle() { return programTargetAngle; }
}
