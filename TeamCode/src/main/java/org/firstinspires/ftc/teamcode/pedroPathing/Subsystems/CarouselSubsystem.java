package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class CarouselSubsystem extends SubsystemBase {
    // 🔧 Constante
    // determinat experimental
    public static final float TICKS_PER_SLOT = 128.1666666f;
    public static final double POWER = 1;
    public static final int POSITION_TOLERANCE = 3;

    // distanță sub care slotul de intrare e ocupat
    public static final double SLOT_OCCUPIED_MM = 100.0;

    // debounce / delay senzor intrare
    private final ElapsedTime entryTimer = new ElapsedTime();
    private boolean timerRunning = false;

    private final DcMotorEx motor;
    private final DistanceSensor entrySensor;

    // 🔧 Coeficienți reglabil prin FTC Dashboard
    public static double P = 15.0;
    public static double kP = 15.0;
    public static double kI = 1;
    public static double kD = 5.0;
    public static double kF = 0;

    private int index = 0;

    // poziție logică: 0,1,2
    private int logicalIndex = 0;

    private int targetPosition = 0;

    // memorie sloturi
    private final boolean[] occupied = new boolean[3];

    public CarouselSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "motorCarusel");
        entrySensor = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocityPIDFCoefficients(kP,kI,kD,kF);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPositionPIDFCoefficients(P);
        motor.setTargetPosition(0);

        // inițial caruselul este gol
        for (int i = 0; i < 3; i++) {
            occupied[i] = false;
        }

    }

    /* ================= MIȘCARE ================= */


    private void moveToIndex(int index) {
        targetPosition = Math.round(index * TICKS_PER_SLOT);
        motor.setTargetPosition(targetPosition);
        motor.setPower(POWER);
    }

    public void stepRight() {
        index++;
        logicalIndex = Math.abs(index % 3);
        moveToIndex(index);
    }

    public void stepLeft() {
        index--;
        logicalIndex = Math.abs(index % 3);
        moveToIndex(index);
    }

    public boolean atTarget() {
        return Math.abs(
                motor.getCurrentPosition() - motor.getTargetPosition()
        ) < POSITION_TOLERANCE;
    }

    /* ================= SLOT DE INTRARE ================= */

    public boolean entrySlotHasBall() {

        boolean raw = entrySensor.getDistance(DistanceUnit.MM) < SLOT_OCCUPIED_MM;

        if (raw) {
            if (!timerRunning) {
                entryTimer.reset();
                timerRunning = true;
            }
            return entryTimer.milliseconds() >= 50;
        } else {
            timerRunning = false;
            return false;
        }
    }



    public boolean isCurrentSlotFree() {
        return !occupied[logicalIndex];
    }

    public void markCurrentSlotOccupied() {
        occupied[logicalIndex] = true;
    }

    public boolean allSlotsOccupied() {
        return occupied[0] && occupied[1] && occupied[2];
    }

    public int getLogicalIndex() {
        return logicalIndex;
    }

    public int getIndex(){
        return index;
    }

    public int getTargetPosition(){
        return targetPosition;
    }

    public int getCurrentPosition(){
        return motor.getCurrentPosition();
    }

    public boolean getOccupied(int i) {
        return occupied[i];
    }
}
