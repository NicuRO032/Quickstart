package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {
    private final DcMotor shooterMotor;
    private final Servo shooterServo;

    private double currentShooterMotorPower = 0.0;
    private double currentShooterServoPosition = 0.0;

    public OuttakeSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setShooterMotorPower(double shooterMotorPower) {
        currentShooterMotorPower = shooterMotorPower;
        shooterMotor.setPower(shooterMotorPower);
    }

    public void setShooterServoPos(double position) {
        shooterServo.setPosition(position);
    }

    /** Returnează puterea curentă (pentru telemetrie) */
    public double getShooterMotorPower() {
         return currentShooterMotorPower;
    }

    public double getShooterServoPosition() {
        currentShooterServoPosition = getShooterServoPosition();
        return currentShooterServoPosition;
    }



    public void stopMotor() {
        shooterMotor.setPower(0);
    }
}
