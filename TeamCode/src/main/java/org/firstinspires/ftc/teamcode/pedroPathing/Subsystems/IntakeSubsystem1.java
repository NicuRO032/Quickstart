package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class IntakeSubsystem1 extends SubsystemBase {
    private final DcMotorEx intakeMotorSus;
    private final DcMotorEx intakeMotorJos;
    private double lastPowerSus = 0, lastPowerJos = 0;

    public enum IntakeState { IDLE, COLLECTING, CLEANUP_BALL3, EJECTING }
    private IntakeState currentState = IntakeState.IDLE;

    public IntakeSubsystem1(HardwareMap hardwareMap) {
        intakeMotorSus = hardwareMap.get(DcMotorEx.class, "motorIntakeSus");
        intakeMotorJos = hardwareMap.get(DcMotorEx.class, "motorIntakeJos");
        intakeMotorSus.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotorJos.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotorJos.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setState(IntakeState state) { this.currentState = state; }

    @Override
    public void periodic() {
        switch (currentState) {
            case IDLE:          applyPowerWithProtection(0, 0); break;
            case COLLECTING:    applyPowerWithProtection(-0.8, -0.8); break;
            case CLEANUP_BALL3: applyPowerWithProtection(-0.8, 0.8); break; // Sus trage, Jos scuipă
            case EJECTING:      applyPowerWithProtection(0.7, 0.7); break;
        }
    }

    private void applyPowerWithProtection(double targetSus, double targetJos) {
        // Protecție: dacă sensul se schimbă față de ultima comandă, punem 0 scurt
        if ((lastPowerSus < 0 && targetSus > 0) || (lastPowerSus > 0 && targetSus < 0)) intakeMotorSus.setPower(0);
        else intakeMotorSus.setPower(targetSus);

        if ((lastPowerJos < 0 && targetJos > 0) || (lastPowerJos > 0 && targetJos < 0)) intakeMotorJos.setPower(0);
        else intakeMotorJos.setPower(targetJos);

        lastPowerSus = targetSus; lastPowerJos = targetJos;
    }

    public void collect() { setState(IntakeState.COLLECTING); }
    public void cleanup() { setState(IntakeState.CLEANUP_BALL3); }
    public void eject()   { setState(IntakeState.EJECTING); }
    public void stop()    { setState(IntakeState.IDLE); }

    public void setPower(double power) { // Pentru override manual
        applyPowerWithProtection(power, power);
        currentState = IntakeState.IDLE;
    }
}