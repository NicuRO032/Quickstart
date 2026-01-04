package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class IntakeSubsystem1 extends SubsystemBase {

    private final DcMotorEx intakeMotor;

    /**
     * Constructor pentru subsistemul de admisie.
     * @param hardwareMap Obiectul hardwareMap de la OpMode.
     */
    public IntakeSubsystem1(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "motorIntake");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setPower(0);
    }

    /**
     * Setează puterea motorului de admisie.
     * O valoare pozitivă va rula motorul într-o direcție (ex: admisie),
     * o valoare negativă în direcția opusă (ex: evacuare).
     *
     * @param power Puterea de setat, între -1.0 și 1.0.
     */
    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    /**
     * Oprește motorul de admisie.
     */
    public void stop() {
        intakeMotor.setPower(0);
    }
}
