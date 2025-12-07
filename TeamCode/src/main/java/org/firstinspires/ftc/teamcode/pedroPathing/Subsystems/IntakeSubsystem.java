package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

// Declarația clasei trebuie să fie prima (după importuri)
public class IntakeSubsystem extends SubsystemBase {

    // Toate variabilele și metodele trebuie să fie AICI, în interiorul clasei

    private final DcMotor brushMotor;
    private final DcMotor beltMotor;

    private double currentBrushMotorPower = 0.0;
    private double currentBeltMotorPower = 0.0;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        brushMotor = hardwareMap.get(DcMotor.class, "brushMotor");
        beltMotor = hardwareMap.get(DcMotor.class, "beltMotor");

        // Inversarea unui motor dacă e necesar
        // NOTĂ: Poți folosi și DcMotorSimple.Direction.REVERSE
        brushMotor.setDirection(DcMotor.Direction.REVERSE);
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

    // --- Metodele de oprire ---

    public void stopBrushMotor() {
        setBrushMotorPower(0);
    }

    public void stopBeltMotor() {
        setBeltMotorPower(0);
    }

    /**
     * O metodă ajutătoare care oprește ambele motoare de la intake.
     * Este bună practică să ai o astfel de metodă.
     */
    public void stop() {
        stopBrushMotor();
        stopBeltMotor();
    }

    // Aici se termină clasa IntakeSubsystem
}
