package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class IntakeSubsystem1 extends SubsystemBase {

    private final DcMotorEx intakeMotor;
    private final DcMotorEx intakeMotor1;

    /**
     * Constructor pentru subsistemul de admisie.
     * @param hardwareMap Obiectul hardwareMap de la OpMode.
     */
    public IntakeSubsystem1(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "motorIntake");
        intakeMotor1 = hardwareMap.get(DcMotorEx.class, "motorIntake1");

        //intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setPower(0);

        intakeMotor1.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor1.setPower(0);
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
        intakeMotor1.setPower(power);
    }

    /**
     * Oprește motorul de admisie.
     */
    public void stop() {
        intakeMotor1.setPower(0);
        intakeMotor.setPower(0);
    }

    public void reverseIntake() {
        // Presupunând că 0.7 scoate bila și -0.8 o trage
        intakeMotor.setPower(-0.8);
        intakeMotor1.setPower(0.8);
    }

}
