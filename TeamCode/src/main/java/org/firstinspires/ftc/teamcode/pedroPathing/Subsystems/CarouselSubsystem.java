package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

public class CarouselSubsystem extends SubsystemBase {

    /* ================= CONSTANTE ================= */

    public static final float TICKS_PER_SLOT = 128.1666666f;
    public static final float OUTTAKE_OFFSET_TICKS = TICKS_PER_SLOT * 1.5f;
    public static final double POWER = 0.4;
    public static final int POSITION_TOLERANCE = 3;
    public static final double SLOT_OCCUPIED_MM = 125.0;
    public static final long SENSOR_DELAY_MS = 200;
    public static final double PUSH_POS = 0.7;
    public static final double RETRACT_POS = 0.2;
    public static final long PUSH_TIME_MS = 1000;

    /* ================= HARDWARE ================= */

    private final DcMotorEx motorCarousel;
    private final DcMotorEx motorShooter;
    private final DistanceSensor entrySensor;
    private final NormalizedColorSensor colorSensor1;
    private final NormalizedColorSensor colorSensor2;

    private final Servo pusher;

    /* ================= INTAKE FSM ================= */

    private enum IntakeState {
        IDLE,
        CHECK_SLOT,
        ROTATE_TO_SLOT,
        STORE_AND_ADVANCE,
        MANUAL_MOVE
    }
    private IntakeState intakeState = IntakeState.IDLE;

    /* ================= OUTTAKE FSM ================= */

    private enum OuttakeState {
        OUT_IDLE, PREPARE_SEQ, ROTATE_TO_OUTTAKE,
        PUSH, WAIT_PUSH, ADVANCE, FINISHED
    }

    private OuttakeState outtakeState = OuttakeState.OUT_IDLE;


    /* ================= LOGIC ================= */

    private int index = 0;
    private int logicalIndex = 0;
    private int targetPosition = 0;

    private final boolean[] occupied = new boolean[3];
    private final BallColor[] slotColor = new BallColor[3];

    private boolean autoEnabled = true;
    private boolean ballHandled = false;
    private boolean autoResumePending = false;

    public int[] outtakeOrder;
    private int outtakePtr = 0;

    public enum OuttakePattern { PGG, GPG, GGP }
    private OuttakePattern pendingOuttakePattern = null;

    /* ================= TIMERS ================= */

    private final ElapsedTime intakeTimer = new ElapsedTime();
    private boolean timerRunning = false;

    private final ElapsedTime outtakeTimer = new ElapsedTime();

    /* ================= COLOR SENSOR================= */
    public enum BallColor {
        GREEN,   // G
        PURPLE,  // P
        UNKNOWN
    }
    final float[] hsvValues1 = new float[3];
    final float[] hsvValues2 = new float[3];

    /* ================= CONSTRUCTOR ================= */

    public CarouselSubsystem(HardwareMap hardwareMap) {

        motorCarousel = hardwareMap.get(DcMotorEx.class, "motorCarusel");
        motorShooter = hardwareMap.get(DcMotorEx.class, "motorShooter");
        entrySensor = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color1");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color2");
        pusher = hardwareMap.get(Servo.class, "pusher");

        motorCarousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorCarousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorCarousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorCarousel.setVelocityPIDFCoefficients(15, 1, 5, 0);
        motorCarousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorCarousel.setPositionPIDFCoefficients(15);
        motorCarousel.setTargetPosition(0);

        for (int i = 0; i < 3; i++) {
            occupied[i] = false;
            slotColor[i] = BallColor.UNKNOWN;
        }
        occupied[0] = true;
        occupied[1] = true;
        occupied[2] = true;
        slotColor[0] = BallColor.PURPLE;
        slotColor[1] = BallColor.GREEN;
        slotColor[2] = BallColor.GREEN;


    }

    /* ================= DISTANCE SENSOR ================= */

    public boolean entrySlotHasBall() {
        boolean raw = entrySensor.getDistance(DistanceUnit.MM) < SLOT_OCCUPIED_MM;

        if (raw) {
            if (!timerRunning) {
                intakeTimer.reset();
                timerRunning = true;
            }
            return intakeTimer.milliseconds() >= SENSOR_DELAY_MS;
        } else {
            timerRunning = false;
            return false;
        }
    }
    /* ================= COLOR SENSOR ================= */
    private BallColor detectBallColor() {
        NormalizedRGBA colors1 = colorSensor1.getNormalizedColors();
        NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();
        Color.colorToHSV(colors1.toColor(), hsvValues1);
        Color.colorToHSV(colors2.toColor(), hsvValues2);
        float hue1 = hsvValues1[0];
        float hue2 = hsvValues2[0];

        float hueMax = Math.max(hue1, hue2);
        //float hueMed = (hue1 + hue2) / 2;

        if (hueMax > 90 && hueMax < 170) {
            return BallColor.GREEN;
        }
        else if (hueMax > 200 && hueMax < 320) {
            return BallColor.PURPLE;
        }
        else {
            return BallColor.UNKNOWN;
        }
    }

    /* ================= MOTOR ================= */

    private void moveToIndexIntake(int idx) {
        targetPosition = Math.round(idx * TICKS_PER_SLOT);
        motorCarousel.setTargetPosition(targetPosition);
        motorCarousel.setPower(POWER);
    }

    private void moveToLogicalIndexOuttake(int idx) {
        if ((logicalIndex == 0) && (idx ==1)){index = index + 1;}
        if ((logicalIndex == 0) && (idx ==2)){index = index - 1;}
        if ((logicalIndex == 1) && (idx ==0)){index = index - 1;}
        if ((logicalIndex == 1) && (idx ==2)){index = index + 1;}
        if ((logicalIndex == 2) && (idx ==0)){index = index + 1;}
        if ((logicalIndex == 2) && (idx ==1)){index = index - 1;}

        logicalIndex = (index % 3 + 3) % 3;

        targetPosition = Math.round(index * TICKS_PER_SLOT - OUTTAKE_OFFSET_TICKS);
        motorCarousel.setTargetPosition(targetPosition);
        motorCarousel.setPower(POWER);
    }

    private boolean atTarget() {
        return Math.abs(motorCarousel.getCurrentPosition() - targetPosition) < POSITION_TOLERANCE;
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
        moveToIndexIntake(index);

        intakeState = IntakeState.MANUAL_MOVE;
    }

    public void manualStepRight() {
        autoEnabled = false;
        autoResumePending = true;

        index++;
        logicalIndex = (index % 3 + 3) % 3;
        moveToIndexIntake(index);

        intakeState = IntakeState.MANUAL_MOVE;
    }

    public boolean allSlotsOccupied() {
        return occupied[0] && occupied[1] && occupied[2];
    }

    /* ================= OUTTAKE LOGIC ================= */

    public void startOuttake(OuttakePattern pattern) {
        if (outtakeState != OuttakeState.OUT_IDLE) return;

        outtakeOrder = buildFallbackOrder(pattern);
        if (outtakeOrder.length == 0) return;

        autoEnabled = false;
        outtakePtr = 0;
        outtakeState = OuttakeState.PREPARE_SEQ;
    }

    private int[] buildFallbackOrder(OuttakePattern pattern) {
        List<Integer> result = new ArrayList<>();
        boolean[] used = new boolean[3];

        BallColor[] wanted;
        switch (pattern) {
            case PGG: wanted = new BallColor[]{BallColor.PURPLE, BallColor.GREEN, BallColor.GREEN}; break;
            case GPG: wanted = new BallColor[]{BallColor.GREEN, BallColor.PURPLE, BallColor.GREEN}; break;
            default:  wanted = new BallColor[]{BallColor.GREEN, BallColor.GREEN, BallColor.PURPLE};
        }

        for (BallColor w : wanted) {
            for (int i = 0; i < 3; i++) {
                if (!used[i] && occupied[i] && slotColor[i] == w) {
                    result.add(i);
                    used[i] = true;
                    break;
                }
            }
        }

        for (int i = 0; i < 3; i++) {
            if (occupied[i] && !used[i]) result.add(i);
        }

        return result.stream().mapToInt(i -> i).toArray();
    }

    /* ================= FSM LOOP ================= */

    @Override
    public void periodic() {
        /* ---------- OUTTAKE FSM ---------- */
        if (outtakeState == OuttakeState.OUT_IDLE) {
        switch (intakeState) {

            case IDLE:
                // Reset only when ball leaves sensor
                if (!entrySlotHasBall()) {
                    ballHandled = false;
                }

                if (autoEnabled && entrySlotHasBall() && !ballHandled && !allSlotsOccupied()) {
                    intakeState = IntakeState.CHECK_SLOT;
                }
                break;

            case CHECK_SLOT:
                if (!occupied[logicalIndex]) {
                    // Slot liber → stochează bila
                    intakeState = IntakeState.STORE_AND_ADVANCE;
                } else {
                    // Slot ocupat → caută următorul
                    index++;
                    logicalIndex = (index % 3 + 3) % 3;
                    moveToIndexIntake(index);
                    intakeState = IntakeState.ROTATE_TO_SLOT;
                }
                break;

            case STORE_AND_ADVANCE:
                if (!ballHandled) {
                    occupied[logicalIndex] = true;
                    slotColor[logicalIndex] = detectBallColor();
                    ballHandled = true;

                    // 🔄 Mută la următorul slot (feedback vizual)
                    index++;
                    logicalIndex = (index % 3 + 3) % 3;
                    moveToIndexIntake(index);

                    intakeState = IntakeState.ROTATE_TO_SLOT;
                }
                break;

            case ROTATE_TO_SLOT:
                if (atTarget()) {
                    intakeState = IntakeState.IDLE;
                }
                break;

            case MANUAL_MOVE:
                if (atTarget()) {
                    if (autoResumePending) {
                        autoEnabled = true;
                        autoResumePending = false;
                    }
                    intakeState = IntakeState.IDLE;
                }
                break;
        }}

        /* ---------- OUTTAKE FSM ---------- */

        switch (outtakeState) {

            case PREPARE_SEQ:
                motorShooter.setPower(0.4);
                outtakeState = OuttakeState.ROTATE_TO_OUTTAKE;
                break;

            case ROTATE_TO_OUTTAKE:
                int slot = outtakeOrder[outtakePtr];
                moveToLogicalIndexOuttake(slot);

                if (atTarget()) {
                    pusher.setPosition(PUSH_POS);
                    outtakeTimer.reset();
                    outtakeState = OuttakeState.PUSH;
                }
                break;

            case PUSH:
                if (outtakeTimer.milliseconds() > PUSH_TIME_MS) {
                    pusher.setPosition(RETRACT_POS);
                    occupied[outtakeOrder[outtakePtr]] = false;
                    slotColor[outtakeOrder[outtakePtr]] = BallColor.UNKNOWN;
                    outtakeState = OuttakeState.ADVANCE;
                }
                break;

            case ADVANCE:
                outtakePtr++;
                if (outtakePtr >= outtakeOrder.length)
                    outtakeState = OuttakeState.FINISHED;
                else
                    outtakeState = OuttakeState.ROTATE_TO_OUTTAKE;
                break;

            case FINISHED:
                motorShooter.setPower(0);
                if (logicalIndex == 1){
                    index--;
                    logicalIndex = (index % 3 + 3) % 3;
                }
                if (logicalIndex == 2){
                    index++;
                    logicalIndex = (index % 3 + 3) % 3;
                }
                moveToIndexIntake(index);
                autoEnabled = true;
                outtakeState = OuttakeState.OUT_IDLE;
                break;
        }
    }


    /* ================= DEBUG ================= */

    public String getIntakeState() {
        return intakeState.name();
    }

    public String getOuttakeState() {
        return outtakeState.name();
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
        return motorCarousel.getCurrentPosition();
    }

    public boolean getOccupied(int i) {
        return occupied[i];
    }

    public BallColor getBallColor(int i){ return slotColor[i];}

    public BallColor getBallColor(){
        return detectBallColor();
    }

    public int getOuttakeOrder(int i){
        return outtakeOrder[i];
    }

    public float getHue1(){
        NormalizedRGBA colors = colorSensor1.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues1);
        return hsvValues1[0];
    }

    public float getHue2(){
        NormalizedRGBA colors = colorSensor2.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues2);
        return hsvValues2[0];
    }
    public float getHueMax(){
        return Math.max(hsvValues1[0] , hsvValues2[0]);
    }

    public double getDistance(){
        return entrySensor.getDistance(DistanceUnit.MM);
    }

}
