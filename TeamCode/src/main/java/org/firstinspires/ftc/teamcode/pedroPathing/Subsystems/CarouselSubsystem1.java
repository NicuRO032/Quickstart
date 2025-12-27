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

public class CarouselSubsystem1 extends SubsystemBase {

    /* ================= CONSTANTE ================= */
    public static final float TICKS_PER_SLOT = 128.1666666f;
    // Decalaj de 180 grade pentru a ajunge de la ora 12 la ora 6
    public static final float OUTTAKE_OFFSET_SLOTS = 1.5f;

    public static final double POWER = 0.4;
    public static final int POSITION_TOLERANCE = 5;
    public static final double SLOT_OCCUPIED_MM = 125.0;
    public static final long SENSOR_DELAY_MS = 200;
    public static final double PUSH_POS = 0.7;
    public static final double RETRACT_POS = 0.2;
    public static final long PUSH_TIME_MS = 1000;
    public static final long RETRACT_TIME_MS = 1000;

    /* ================= HARDWARE ================= */
    private final DcMotorEx motorCarousel, motorShooter;
    private final DistanceSensor entrySensor;
    private final NormalizedColorSensor colorSensor1, colorSensor2;
    private final Servo pusher;

    /* ================= STATES ================= */
    public enum IntakeState { IDLE, CHECK_SLOT, ROTATE_TO_SLOT, STORE_AND_ADVANCE, MANUAL_MOVE }
    private IntakeState intakeState = IntakeState.IDLE;

    public enum OuttakeState { OUT_IDLE, PREPARE_READY, PUSH, WAIT_RETRACT, ADVANCE, FINISHED }
    private OuttakeState outtakeState = OuttakeState.OUT_IDLE;

    /* ================= LOGIC ================= */
    private int globalIndex = 0; // Indexul cumulativ pentru Shortest Path
    private int logicalIndex = 0; // Slotul aflat în dreptul orei 12 (0, 1 sau 2)
    private int targetPosition = 0;

    private final boolean[] occupied = new boolean[3];
    private final BallColor[] slotColor = new BallColor[3];
    private boolean autoEnabled = true, ballHandled = false;

    private int[] outtakeOrder = new int[0];
    private int outtakePtr = 0;
    private boolean triggerReady = false;

    public enum BallColor { GREEN, PURPLE, UNKNOWN }
    public enum OuttakePattern { PGG, GPG, GGP }
    private OuttakePattern activePattern = OuttakePattern.PGG;

    private final ElapsedTime intakeTimer = new ElapsedTime();
    private final ElapsedTime outtakeTimer = new ElapsedTime();
    private boolean timerRunning = false;

    public CarouselSubsystem1(HardwareMap hardwareMap) {
        motorCarousel = hardwareMap.get(DcMotorEx.class, "motorCarusel");
        motorShooter = hardwareMap.get(DcMotorEx.class, "motorShooter");
        entrySensor = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color1");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color2");
        pusher = hardwareMap.get(Servo.class, "pusher");

        motorCarousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMPORTANT: Întâi punem puterea 0, apoi resetăm
        motorCarousel.setPower(0);
        motorCarousel.setTargetPosition(0);
        motorCarousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // NU trece în RUN_TO_POSITION aici dacă vrei liniște totală.
        // Treci în RUN_USING_ENCODER și activează RUN_TO_POSITION doar în resetForStart()
        motorCarousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        for (int i = 0; i < 3; i++) { occupied[i] = false; slotColor[i] = BallColor.UNKNOWN; }
        pusher.setPosition(RETRACT_POS);
    }

    /* ================= CORE LOGIC: SHORTEST PATH ================= */

    /**
     * Metodă universală de poziționare.
     * @param targetSlot Slotul (0, 1, 2) pe care îl vrem la destinație.
     * @param isOuttake Daca este true, slotul merge la ora 6. Dacă e false, la ora 12.
     */
    private void goToSlot(int targetSlot, boolean isOuttake) {
        // 1. Calculăm unde se află ACUM motorul în termeni de "sloturi logice"
        // Această valoare ne spune la ce slot (0, 1, 2) corespunde poziția curentă a motorului
        double currentPhysicalPosInSlots = (double)motorCarousel.getCurrentPosition() / TICKS_PER_SLOT;

        // 2. Calculăm poziția țintă absolută pentru slotul dorit
        // Dacă e outtake, slotul trebuie să ajungă la ora 6 (deci -1.5 sloturi față de referința orei 12)
        double targetPhysicalPosInSlots = targetSlot;
        if (isOuttake) {
            targetPhysicalPosInSlots -= OUTTAKE_OFFSET_SLOTS;
        }

        // 3. Calculăm diferența față de rotația actuală (globalIndex)
        // Vrem să aflăm câte sloturi trebuie să ne mișcăm față de UNDE SUNTEM ACUM
        double currentGlobalBase = (double)globalIndex;
        if (outtakeState != OuttakeState.OUT_IDLE) {
            // Dacă suntem în outtake, baza curentă a fost deja decalată
            currentGlobalBase -= OUTTAKE_OFFSET_SLOTS;
        }

        // Diferența logică de sloturi
        int currentLogicalAt12 = (globalIndex % 3 + 3) % 3;
        int diff = targetSlot - currentLogicalAt12;

        // Normalizare pentru cea mai scurtă cale
        if (diff > 1) diff -= 3;
        else if (diff < -1) diff += 3;

        // Actualizăm indexul global
        globalIndex += diff;

        // 4. Calculăm tick-urile finale
        float totalSlotsToMove = (float)globalIndex;
        if (isOuttake) {
            totalSlotsToMove -= OUTTAKE_OFFSET_SLOTS;
        }

        targetPosition = Math.round(totalSlotsToMove * TICKS_PER_SLOT);
        motorCarousel.setTargetPosition(targetPosition);
        motorCarousel.setPower(POWER);

        logicalIndex = targetSlot;
    }

    /* ================= API CONTROL ================= */
    public void resetForStart() {
        // --- RESET HARDWARE ---
        motorCarousel.setPower(0);
        motorCarousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorCarousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorCarousel.setTargetPosition(0);
        motorCarousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // --- RESET LOGICĂ ---
        this.globalIndex = 0;
        this.logicalIndex = 0;
        this.targetPosition = 0;

        // Asigurăm stările de repaus
        this.intakeState = IntakeState.IDLE;
        this.outtakeState = OuttakeState.OUT_IDLE;

        // Resetăm flag-urile de control
        this.autoEnabled = false; //incepem autonomia cu controlul la outtake, pentru teleop apelam activateIntake dupa resetForStart
        this.ballHandled = false;
        this.triggerReady = false;
    }

    public void activateIntake() {
        autoEnabled = true;
    }
    public void deactivateIntake() {
        autoEnabled = false;
    }

    public void forcePreload(BallColor s0, BallColor s1, BallColor s2) {
        occupied[0] = true; slotColor[0] = s0;
        occupied[1] = true; slotColor[1] = s1;
        occupied[2] = true; slotColor[2] = s2;
        //autoEnabled = false;
        //this.intakeState = IntakeState.IDLE; // Ne asigurăm că nu pleacă singur la pornire
    }

    // --- Creează o metodă pentru a seta pattern-ul din exterior ---
    public void setActivePattern(OuttakePattern pattern) {
        this.activePattern = pattern;
    }

    public void prepareOuttake(OuttakePattern pattern) {
        if (outtakeState != OuttakeState.OUT_IDLE) return;
        outtakeOrder = buildFallbackOrder(pattern);
        if (outtakeOrder.length == 0) return;

        autoEnabled = false;
        outtakePtr = 0;
        triggerReady = false;
        motorShooter.setPower(0.5);
        outtakeState = OuttakeState.PREPARE_READY;
    }

    public void triggerShoot() {
        if (outtakeState == OuttakeState.PREPARE_READY && atTarget()) triggerReady = true;
    }

    public void abortAll() {
        outtakeState = OuttakeState.OUT_IDLE;
        intakeState = IntakeState.IDLE;
        motorShooter.setPower(0);
        pusher.setPosition(RETRACT_POS);
        autoEnabled = true;
        triggerReady = false;
        goToSlot(0, false); // Revenire la bază
    }

    public void manualStepLeft() {
        autoEnabled = false;
        int nextSlot = (logicalIndex - 1 + 3) % 3;
        goToSlot(nextSlot, false);
        intakeState = IntakeState.MANUAL_MOVE;
    }

    public void manualStepRight() {
        autoEnabled = false;
        int nextSlot = (logicalIndex + 1) % 3;
        goToSlot(nextSlot, false);
        intakeState = IntakeState.MANUAL_MOVE;
    }

    /* ================= FSM LOOP ================= */

    @Override
    public void periodic() {
        if ((outtakeState == OuttakeState.OUT_IDLE)&&autoEnabled) handleIntake();
        handleOuttake();

        // TRANZIȚIE AUTOMATĂ: Folosește activePattern-ul salvat
        if (allSlotsOccupied() && outtakeState == OuttakeState.OUT_IDLE && autoEnabled) {
            prepareOuttake(activePattern);
        }
    }

    private void handleIntake() {
        switch (intakeState) {
            case IDLE:
                if (!entrySlotHasBall()) ballHandled = false;
                if (autoEnabled && entrySlotHasBall() && !ballHandled && !allSlotsOccupied())
                    intakeState = IntakeState.CHECK_SLOT;
                break;
            case CHECK_SLOT:
                if (!occupied[logicalIndex]) intakeState = IntakeState.STORE_AND_ADVANCE;
                else {
                    goToSlot((logicalIndex + 1) % 3, false);
                    intakeState = IntakeState.ROTATE_TO_SLOT;
                }
                break;
            case STORE_AND_ADVANCE:
                occupied[logicalIndex] = true;
                slotColor[logicalIndex] = detectBallColor();
                ballHandled = true;
                goToSlot((logicalIndex + 1) % 3, false);
                intakeState = IntakeState.ROTATE_TO_SLOT;
                break;
            case ROTATE_TO_SLOT:
            case MANUAL_MOVE:
                if (atTarget()) {
                    autoEnabled = true;
                    intakeState = IntakeState.IDLE;
                }
                break;
        }
    }

    private void handleOuttake() {
        switch (outtakeState) {
            case PREPARE_READY:
                goToSlot(outtakeOrder[outtakePtr], true);
                if (atTarget() && triggerReady) {
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
                    outtakeTimer.reset();
                    outtakeState = OuttakeState.WAIT_RETRACT;
                }
                break;

            case WAIT_RETRACT:
                if (outtakeTimer.milliseconds() > RETRACT_TIME_MS) {
                    outtakePtr++; // Trecem la următoarea bilă din listă
                    if (outtakePtr >= outtakeOrder.length) {
                        outtakeState = OuttakeState.FINISHED;
                    } else {
                        // Mergem la următoarea bilă
                        goToSlot(outtakeOrder[outtakePtr], true);
                        outtakeState = OuttakeState.ADVANCE;
                    }
                }
                break;

            case ADVANCE:
                // Aici așteptăm ca motorul să ajungă la noua bilă
                if (atTarget()) {
                    pusher.setPosition(PUSH_POS);
                    outtakeTimer.reset();
                    outtakeState = OuttakeState.PUSH;
                }
                break;

            case FINISHED:
                motorShooter.setPower(0);
                triggerReady = false;
                // Forțăm întoarcerea la ora 12 pentru slotul 0
                goToSlot(0, false);
                if (atTarget()) {
                    autoEnabled = true;
                    outtakeState = OuttakeState.OUT_IDLE;
                }
                break;
        }
    }

    /* ================= HELPERS (Detectie Culoare, Senzori etc.) ================= */

    private int[] buildFallbackOrder(OuttakePattern pattern) {
        List<Integer> result = new ArrayList<>();
        boolean[] used = new boolean[3];
        BallColor[] wanted;
        if (pattern == OuttakePattern.PGG) wanted = new BallColor[]{BallColor.PURPLE, BallColor.GREEN, BallColor.GREEN};
        else if (pattern == OuttakePattern.GPG) wanted = new BallColor[]{BallColor.GREEN, BallColor.PURPLE, BallColor.GREEN};
        else wanted = new BallColor[]{BallColor.GREEN, BallColor.GREEN, BallColor.PURPLE};

        for (BallColor w : wanted) {
            for (int i = 0; i < 3; i++) {
                if (!used[i] && occupied[i] && slotColor[i] == w) {
                    result.add(i); used[i] = true; break;
                }
            }
        }
        for (int i = 0; i < 3; i++) if (occupied[i] && !used[i]) result.add(i);
        return result.stream().mapToInt(i -> i).toArray();
    }

    private BallColor detectBallColor() {
        NormalizedRGBA c1 = colorSensor1.getNormalizedColors();
        NormalizedRGBA c2 = colorSensor2.getNormalizedColors();
        float[] hsv1 = new float[3], hsv2 = new float[3];
        Color.colorToHSV(c1.toColor(), hsv1);
        Color.colorToHSV(c2.toColor(), hsv2);
        float hue = Math.max(hsv1[0], hsv2[0]);
        if (hue > 90 && hue < 170) return BallColor.GREEN;
        if (hue > 200 && hue < 320) return BallColor.PURPLE;
        return BallColor.UNKNOWN;
    }

    public boolean entrySlotHasBall() {
        boolean raw = entrySensor.getDistance(DistanceUnit.MM) < SLOT_OCCUPIED_MM;
        if (raw) {
            if (!timerRunning) { intakeTimer.reset(); timerRunning = true; }
            return intakeTimer.milliseconds() >= SENSOR_DELAY_MS;
        } else { timerRunning = false; return false; }
    }

    /* ================= GETTERS ================= */
    public boolean isReadyToShoot() { return outtakeState == OuttakeState.PREPARE_READY && atTarget(); }
    public OuttakePattern getActivePattern() {return this.activePattern;}
    public boolean atTarget() { return Math.abs(motorCarousel.getCurrentPosition() - targetPosition) < POSITION_TOLERANCE; }
    public boolean allSlotsOccupied() { return occupied[0] && occupied[1] && occupied[2]; }
    public String getIntakeState() { return intakeState.name(); }
    public String getOuttakeState() { return outtakeState.name(); }
    public int getLogicalIndex() { return logicalIndex; }
    public int getGlobalIndex() { return globalIndex; }
    public int getTargetPosition() { return targetPosition; }
    public int getCurrentPosition() { return motorCarousel.getCurrentPosition(); }
    public boolean getOccupied(int i) { return occupied[i]; }
    public BallColor getBallColor(int i) { return slotColor[i]; }

    public String getSlotsColorString() {
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < 3; i++) {
            if (!occupied[i]) sb.append("Empty");
            else {
                switch (slotColor[i]) {
                    case GREEN:  sb.append("Green"); break;
                    case PURPLE: sb.append("Purple"); break;
                    default:     sb.append("?"); break;
                }
            }
            if (i < 2) sb.append(", ");
        }
        return sb.append("]").toString();
    }
    public String getOuttakeOrderString() {
        if (outtakeOrder == null || outtakeOrder.length == 0) return "EMPTY";
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < outtakeOrder.length; i++) {
            sb.append(outtakeOrder[i]);
            if (i < outtakeOrder.length - 1) sb.append(", ");
        }
        return sb.append("]").toString();
    }
    public int getOuttakePtr() { return outtakePtr; }
}