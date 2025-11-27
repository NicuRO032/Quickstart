package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotor brushMotor;
    private final DcMotor beltMotor;

    private double currentBrushMotorPower = 0.0;
    private double currentBeltMotorPower = 0.0;


    public IntakeSubsystem(HardwareMap hardwareMap) {
        brushMotor = hardwareMap.get(DcMotor.class, "brushMotor");
        beltMotor = hardwareMap.get(DcMotor.class, "beltMotor");

        // inversarea unui motor dacă e necesar
        brushMotor.setDirection(DcMotor.Direction.FORWARD);
        beltMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setBrushMotorPower(double brushMotorPower) {
        currentBrushMotorPower = brushMotorPower;
        brushMotor.setPower(brushMotorPower);
    }

    public void setBeltMotorPower(double beltMotorPower) {
        currentBeltMotorPower = beltMotorPower;
        beltMotor.setPower(beltMotorPower);
    }

    /** Returnează puterea curentă (pentru telemetrie) */
    public double getBrushMotorPower() {
        return currentBrushMotorPower;
    }

    public double getBeltMotorPower() {
        return currentBeltMotorPower;
    }

    public void stopBrushMotor() {
        currentBrushMotorPower = 0;
        setBrushMotorPower(0);
    }

    public void stopBeltMotor() {
        currentBeltMotorPower = 0;
        setBeltMotorPower(0);
    }

}
