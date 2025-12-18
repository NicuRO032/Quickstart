package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class CarouselSubsystem extends SubsystemBase {

    /* ================= CONSTANTE ================= */

    public static final float TICKS_PER_SLOT = 128.1666666f;
    public static final double POWER = 0.7;
    public static final int POSITION_TOLERANCE = 3;
    public static final double SLOT_OCCUPIED_MM = 100.0;
    public static final long SENSOR_DELAY_MS = 200;

    /* ================= HARDWARE ================= */

    private final DcMotorEx motor;
    private final DistanceSensor entrySensor;

    /* ================= FSM ================= */

    private enum State {
        IDLE,
        CHECK_SLOT,
        ROTATE_TO_SLOT,
        STORE_AND_ADVANCE,
        MANUAL_MOVE
    }

    private State state = State.IDLE;

    /* ================= LOGIC ================= */

    private int index = 0;
    private int logicalIndex = 0;
    private int targetPosition = 0;

    private final boolean[] occupied = new boolean[3];

    private boolean autoEnabled = true;
    private boolean ballHandled = false;
    private boolean autoResumePending = false;

    /* ================= SENSOR FILTER ================= */

    private final ElapsedTime entryTimer = new ElapsedTime();
    private boolean timerRunning = false;

    /* ================= CONSTRUCTOR ================= */

    public CarouselSubsystem(HardwareMap hardwareMap) {

        motor = hardwareMap.get(DcMotorEx.class, "motorCarusel");
        entrySensor = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor.setVelocityPIDFCoefficients(15, 1, 5, 0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPositionPIDFCoefficients(15);
        motor.setTargetPosition(0);

        for (int i = 0; i < 3; i++) {
            occupied[i] = false;
        }
    }

    /* ================= SENSOR ================= */

    public boolean entrySlotHasBall() {
        boolean raw = entrySensor.getDistance(DistanceUnit.MM) < SLOT_OCCUPIED_MM;

        if (raw) {
            if (!timerRunning) {
                entryTimer.reset();
                timerRunning = true;
            }
            return entryTimer.milliseconds() >= SENSOR_DELAY_MS;
        } else {
            timerRunning = false;
            return false;
        }
    }

    /* ================= MOTOR ================= */

    private void moveToIndex(int idx) {
        targetPosition = Math.round(idx * TICKS_PER_SLOT);
        motor.setTargetPosition(targetPosition);
        motor.setPower(POWER);
    }

    private boolean atTarget() {
        return Math.abs(motor.getCurrentPosition() - targetPosition) < POSITION_TOLERANCE;
    }

    /* ================= API PUBLIC ================= */

    public void enableAuto(boolean enabled) {
        autoEnabled = enabled;
    }

    public void manualStepLeft() {
        autoEnabled = false;
        autoResumePending = true;

        index--;
        logicalIndex = (index % 3 + 3) % 3;
        moveToIndex(index);

        state = State.MANUAL_MOVE;
    }

    public void manualStepRight() {
        autoEnabled = false;
        autoResumePending = true;

        index++;
        logicalIndex = (index % 3 + 3) % 3;
        moveToIndex(index);

        state = State.MANUAL_MOVE;
    }

    public boolean allSlotsOccupied() {
        return occupied[0] && occupied[1] && occupied[2];
    }

    /* ================= FSM LOOP ================= */

    @Override
    public void periodic() {

        switch (state) {

            case IDLE:
                // Reset only when ball leaves sensor
                if (!entrySlotHasBall()) {
                    ballHandled = false;
                }

                if (autoEnabled && entrySlotHasBall() && !ballHandled && !allSlotsOccupied()) {
                    state = State.CHECK_SLOT;
                }
                break;

            case CHECK_SLOT:
                if (!occupied[logicalIndex]) {
                    // Slot liber → stochează bila
                    state = State.STORE_AND_ADVANCE;
                } else {
                    // Slot ocupat → caută următorul
                    index++;
                    logicalIndex = (index % 3 + 3) % 3;
                    moveToIndex(index);
                    state = State.ROTATE_TO_SLOT;
                }
                break;

            case STORE_AND_ADVANCE:
                if (!ballHandled) {
                    occupied[logicalIndex] = true;
                    ballHandled = true;

                    // 🔄 Mută la următorul slot (feedback vizual)
                    index++;
                    logicalIndex = (index % 3 + 3) % 3;
                    moveToIndex(index);

                    state = State.ROTATE_TO_SLOT;
                }
                break;

            case ROTATE_TO_SLOT:
                if (atTarget()) {
                    state = State.IDLE;
                }
                break;

            case MANUAL_MOVE:
                if (atTarget()) {
                    if (autoResumePending) {
                        autoEnabled = true;
                        autoResumePending = false;
                    }
                    state = State.IDLE;
                }
                break;
        }
    }

    /* ================= DEBUG ================= */

    public String getState() {
        return state.name();
    }

    public int getIndex() {
        return index;
    }

    public int getLogicalIndex() {
        return logicalIndex;
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public boolean getOccupied(int i) {
        return occupied[i];
    }
}
