package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {
    private final DcMotor motor;
    private final Servo servo;

    private double currentPower = 0.0;
    private double currentPosition = 0.0;

    public OuttakeSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "outtakeMotor");
        servo = hardwareMap.get(Servo.class, "outtakeServo");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runMotor(double power) {
        currentPower = power;
        motor.setPower(power);
    }

    /** Returnează puterea curentă (pentru telemetrie) */
    public double getPower() {
        return currentPower;
    }

    public double getServoPosition() {
        currentPosition = getServoPosition();
        return currentPosition;
    }

    public void setServo(double position) {
        servo.setPosition(position);
    }

    public void stopMotor() {
        motor.setPower(0);
    }
}
