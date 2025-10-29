package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotor leftMotor;
    private final DcMotor rightMotor;

    private double currentPower = 0.0;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, "intakeLeft");
        rightMotor = hardwareMap.get(DcMotor.class, "intakeRight");

        // inversarea unui motor dacă e necesar
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setPower(double power) {
        currentPower = power;
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    /** Returnează puterea curentă (pentru telemetrie) */
    public double getPower() {
        return currentPower;
    }

    public void stop() {
        setPower(0);
    }
}
