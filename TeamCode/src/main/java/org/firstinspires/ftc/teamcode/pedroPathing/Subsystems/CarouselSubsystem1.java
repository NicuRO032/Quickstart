package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.ArrayList;
import java.util.List;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.TeleOpCarousel1.intakeIsOn;

import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;

//import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

@Config
public class CarouselSubsystem1 extends SubsystemBase {
    /* ================= CONSTANTE CARUSEL SERVO ================= */
    // Vectori pentru pozițiile de Intake (servo) și Outtake (servo)
    private static final double[] INTAKE_POSITIONS = {0.156, 0.414, 0.666};
    private static final double[] OUTTAKE_POSITIONS = {0.542, 0.796, 0.286};

    // Vectori pentru valorile de feedback corespunzătoare (în mV)
    private static final double[] INTAKE_FEEDBACK_MV = {663.0, 1400.0, 2112.0};
    private static final double[] OUTTAKE_FEEDBACK_MV = {1760.0, 2483.0, 1032.0};

    // Toleranța pentru atTarget, în milivolți (mV)
    // Crește toleranța: cu cât e mai mare, cu atât consideră mai repede că "a ajuns"
    public static double FEEDBACK_TOLERANCE_MV = 120; // de la 50.0

    // Scade timpul de așteptare: 50ms e mult în competiție
    public static long AT_TARGET_STABILITY_MS = 20; // de la 50 // Timpul de stabilitate (păstrat)

    // shooter
    public static double SHOOTER_kP = 0.001;
    public static double SHOOTER_kI = 0.0;
    public static double SHOOTER_kD = 0.000001;
    public static double SHOOTER_kF = 0.00045;

    public static final double SHOOTER_MOTOR_CPR = 28.0;
    public static double DEFAULT_SHOOTER_RPM = 4500.0;
    public static double SHOT_CONFIRM_DIP_PERCENT = 0.05; // Acum se aplică la RPM
    private double rpmBeforePush = 0.0;
    private boolean shotWasDetected = false;
    private final PIDController shooterController;

    private double currentTargetRPM = 0.0; // Ținta pentru shooter, în RPM




    /* ================= CONSTANTE ================= */

    public static final double SLOT_OCCUPIED_MM = 100.0;
    public static double COLOR_SENSOR_OCCUPIED_MM = 70.0;
    public static final long SENSOR_DELAY_MS = 10;
    public static final double PUSH_POS = 0.1;
    public static final double RETRACT_POS = 0.5;
    public static long PUSH_TIME_MS = 250;
    public static long RETRACT_TIME_MS = 125;
    public static int MAX_SHOT_RETRIES = 1; // Permitem o singură reîncercare suplimentară
    private int shotRetryCounter = 0;


    /* ================= HARDWARE ================= */
    private final DcMotorEx shooterMotor;
    private final Servo carouselServo;
    private final AnalogInput carouselFeedback;
    private final DistanceSensor entrySensor;
    private final NormalizedColorSensor colorSensor1, colorSensor2;
    private final Servo pusher;
    private final DigitalChannel pusherMagnetSensor;

    private final VoltageSensor batteryVoltageSensor;

    /* ================= STATES ================= */
    public enum IntakeState {IDLE, STORE_AND_ADVANCE, MANUAL_MOVE}
    private IntakeState intakeState = IntakeState.IDLE;

    public enum OuttakeState { OUT_IDLE, PREPARE_READY, ALIGNING_FOR_SHOT, PUSH, CONFIRM_SHOT, ADVANCE, FINISHED }
    private OuttakeState outtakeState = OuttakeState.OUT_IDLE;

    /* ================= LOGIC ================= */

    private int logicalIndex = 0; // Indexul slotului (0, 1, 2)
    private double targetServoPosition = 0.0; // Poziția țintă pentru servo
    private double targetFeedbackMv = 0.0; // Valoarea de feedback (mV) așteptată la țintă
    private final ElapsedTime atTargetTimer = new ElapsedTime(); //Cronometru pentru atTarget

    private final boolean[] occupied = new boolean[3];
    public static final BallColor[] slotColor = new BallColor[3];
    private boolean autoEnabled = true, ballHandled = false;

    private int[] outtakeOrder = new int[0];
    private int outtakePtr = 0;
    private boolean triggerReady = false;
    public boolean isTeleOp = false;


    public enum BallColor { GREEN, PURPLE, UNKNOWN }
    public enum OuttakePattern { GPP, PGP, PPG }
    private OuttakePattern activePattern = OuttakePattern.GPP;

    private final ElapsedTime intakeTimer = new ElapsedTime();
    private final ElapsedTime outtakeTimer = new ElapsedTime();
    private boolean timerRunning = false;

    final float[] hsvValues1 = new float[3];
    final float[] hsvValues2 = new float[3];

    public CarouselSubsystem1(HardwareMap hardwareMap) {


        carouselServo = hardwareMap.get(Servo.class, "carouselServo");
        carouselFeedback = hardwareMap.get(AnalogInput.class, "axonFeedback");

        shooterMotor = hardwareMap.get(DcMotorEx.class, "motorShooter");
        entrySensor = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color1");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color2");
        pusher = hardwareMap.get(Servo.class, "pusher");
        //pusherMagnetSensor = hardwareMap.get(DigitalChannel.class, "magnet");
        pusherMagnetSensor = hardwareMap.get(DigitalChannel.class, "magnet");
        pusherMagnetSensor.setMode(DigitalChannel.Mode.INPUT);


        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();



        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterController = new PIDController(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD);



        for (int i = 0; i < 3; i++) { occupied[i] = false; slotColor[i] = BallColor.UNKNOWN; }
        pusher.setPosition(RETRACT_POS);
    }

    // --- Funcții de conversie pentru shooter ---
    private double rpmToTicksPerSecond(double rpm) {
        return (rpm / 60.0) * SHOOTER_MOTOR_CPR;
    }
    private double ticksPerSecondToRpm(double ticksPerSecond) {
        return (ticksPerSecond / SHOOTER_MOTOR_CPR) * 60.0;
    }

    // --- Settere și Gettere ---
    public void setShooterTargetRPM(double rpm) { this.currentTargetRPM = rpm; }// Metodă publică pentru a seta viteza shooter-ului din exterior
    public void setShooterForAutoRPM(double rpm) {this.DEFAULT_SHOOTER_RPM = rpm;}
    public double getShooterTargetRPM() { return this.currentTargetRPM; }
    public double getShooterCurrentRPM() { return ticksPerSecondToRpm(shooterMotor.getVelocity()); }
    public double getRpmBeforePush() { return rpmBeforePush; }
    public boolean isPusherRetracted() {
        // Senzorii digitali (Hall effect) de obicei returnează 'false' când magnetul este prezent.
        // Verifică acest comportament; s-ar putea să fie nevoie să inversezi logica (!pusherMagnetSensor.getState()).
        return !pusherMagnetSensor.getState();
    }


    private void goToSlot(int targetSlot, boolean isOuttake) {
        if (targetSlot < 0 || targetSlot > 2) return; // Siguranță

        logicalIndex = targetSlot; // Actualizăm indexul logic

        if (isOuttake) {
            targetServoPosition = OUTTAKE_POSITIONS[targetSlot];
            targetFeedbackMv = OUTTAKE_FEEDBACK_MV[targetSlot];
        } else {
            targetServoPosition = INTAKE_POSITIONS[targetSlot];
            targetFeedbackMv = INTAKE_FEEDBACK_MV[targetSlot];
        }

        carouselServo.setPosition(targetServoPosition); // Comandă mișcarea servoului
        atTargetTimer.reset(); // Resetăm cronometrul de stabilitate
    }



    public void resetForStart() {
        goToSlot(0, false);
        this.logicalIndex = 0;
        this.intakeState = IntakeState.IDLE;
        this.outtakeState = OuttakeState.OUT_IDLE;
        this.autoEnabled = true;
        this.ballHandled = false;
        this.triggerReady = false;
    }

    public void activateIntake() { autoEnabled = true; }
    public void deactivateIntake() { autoEnabled = false; }

    public void forcePreload(BallColor s0, BallColor s1, BallColor s2) {
        occupied[0] = true; slotColor[0] = s0;
        occupied[1] = true; slotColor[1] = s1;
        occupied[2] = true; slotColor[2] = s2;
    }

    public void setActivePattern(OuttakePattern pattern) { this.activePattern = pattern; }

    public void prepareOuttake(OuttakePattern pattern) {
        if (outtakeState != OuttakeState.OUT_IDLE) return;
        outtakeOrder = buildFallbackOrder(pattern);
        if (outtakeOrder.length == 0) return;
        autoEnabled = false;
        outtakePtr = 0;
        triggerReady = false;
        setShooterTargetRPM(DEFAULT_SHOOTER_RPM);
        outtakeState = OuttakeState.PREPARE_READY;
    }

    public void triggerShoot() {
        // Permitem declanșarea dacă suntem în starea de aliniere și caruselul s-a oprit la țintă.
        if (outtakeState == OuttakeState.ALIGNING_FOR_SHOT && atTarget()) {
            triggerReady = true;
        }
    }
    public void abortAll() {
        outtakeState = OuttakeState.OUT_IDLE;
        intakeState = IntakeState.IDLE;
        setShooterTargetRPM(0.0);
        pusher.setPosition(RETRACT_POS);
        autoEnabled = true;
        triggerReady = false;
        goToSlot(0, false);
    }

    public void manualStepLeft() {
        autoEnabled = false;
        int nextSlot = (logicalIndex - 1 + 3) % 3;
        goToSlot(nextSlot, false);
        intakeState = IntakeState.MANUAL_MOVE;
    }

    public void manualStepRight() {////////////Aici trebuie refacut true-->false si logivalIndex+1
        autoEnabled = false;
        int nextSlot = (logicalIndex ) % 3;
        goToSlot(nextSlot, true);
        intakeState = IntakeState.MANUAL_MOVE;
    }


    private void handleIntake() {
        switch (intakeState) {
            /**
             * STAREA 1: Așteaptă sosirea unei bile.
             * Stăm aici până când o bilă este detectată stabil la intrare ȘI caruselul nu e plin.
             */
            case IDLE:
                // Condiția de pornire: este activat modul automat, o bilă a sosit și mai este loc.
                if (autoEnabled && entrySlotHasBall() && !allSlotsOccupied()) {
                    // O bilă a sosit. Trecem la pasul 2: stocare și avansare.
                    intakeState = IntakeState.STORE_AND_ADVANCE;
                }
                break;

            /**
             * STAREA 2: Marchează slotul, citește culoarea și avansează la următorul.
             * Această stare se execută o singură dată per bilă.
             */
            case STORE_AND_ADVANCE:
                // a) Marcăm slotul curent (logicalIndex) ca fiind ocupat.
                occupied[logicalIndex] = true;

                // b) Citim și salvăm culoarea bilei.
                slotColor[logicalIndex] = detectBallColor();

                // Verificăm dacă am umplut caruselul DUPĂ ce am adăugat bila curentă.
                if (allSlotsOccupied()) {
                    /**
                     * STAREA 3: Toate sloturile sunt ocupate.
                     * Pregătim mașina de stări pentru outtake.
                     */
                    prepareOuttake(activePattern);
                    intakeIsOn = false;
                    // După pregătire, ne întoarcem la IDLE. Intake-ul va fi oricum dezactivat
                    // de către 'prepareOuttake' (prin autoEnabled = false).
                    intakeState = IntakeState.IDLE;

                } else {
                    // Mai este loc. Găsim următorul slot liber.
                    int nextEmptySlot = -1;
                    for (int i = 0; i < 3; i++) {
                        // Căutăm pornind de la slotul următor celui curent, pentru eficiență.
                        int checkIndex = (logicalIndex + 1 + i) % 3;
                        if (!occupied[checkIndex]) {
                            nextEmptySlot = checkIndex;
                            break;
                        }
                    }

                    // Dacă am găsit un slot gol (ceea ce ar trebui să se întâmple mereu aici),
                    // comandăm rotirea caruselului pentru a-l aduce la ora 12.
                    if (nextEmptySlot != -1) {
                        goToSlot(nextEmptySlot, false);
                    }

                    // IMPORTANT: După ce am comandat mișcarea, ne întoarcem IMEDIAT la IDLE.
                    // Mașina de stări de intake și-a terminat treaba pentru această bilă.
                    // Acum așteaptă dispariția bilei curente și apariția uneia noi.
                    intakeState = IntakeState.IDLE;
                }
                break;


            /**
             * STAREA 4: Control Manual.
             * Această stare este activată de funcțiile manualStep. Așteaptă finalizarea
             * mișcării și apoi reactivează automat intake-ul.
             */
            case MANUAL_MOVE:
                if (atTarget()) {
                    intakeState = IntakeState.IDLE;
                    autoEnabled = true;
                }
                break;
        }
    }

    private void handleOuttake() {
        double RPMdif = Math.abs(getShooterCurrentRPM() - Math.min(getShooterTargetRPM(), 6000));
        double RPMallowedDif = 200;
        switch (outtakeState) {
            case OUT_IDLE:
                // Stare de repaus, nu se face nimic.
                break;

            case PREPARE_READY:
                // 1. Comandăm mișcarea caruselului spre primul slot din secvență.
                goToSlot(outtakeOrder[outtakePtr], true);
                // 2. Trecem IMEDIAT la starea de așteptare a aliniamentului.
                outtakeState = OuttakeState.ALIGNING_FOR_SHOT;
                break;

            case ALIGNING_FOR_SHOT:
                // Așteptăm ca și caruselul să ajungă la țintă, shooter-ul să fie la turație ȘI pilotul să apese pe trăgaci.
                if (atTarget() && triggerReady && RPMdif <= RPMallowedDif) {
                    // Când toate condițiile sunt îndeplinite, suntem gata de aruncare.

                    // a) Salvăm datele necesare pentru confirmarea loviturii
                    rpmBeforePush = getShooterCurrentRPM();
                    shotWasDetected = false;
                    triggerReady = false; // Consumăm trigger-ul pentru a nu trage în buclă

                    // b) Comandăm împingerea
                    pusher.setPosition(PUSH_POS);
                    outtakeTimer.reset(); // Pornim timer-ul pentru PUSH_TIME_MS

                    // c) Trecem la starea de împingere
                    outtakeState = OuttakeState.PUSH;
                }
                break;

            /**
             * STARE MODIFICATĂ: PUSH
             * Gestionează atât împingerea (bazată pe timp), cât și așteptarea retragerii (bazată pe senzor).
             */
            case PUSH:
                // Pasul 1: În timp ce pusher-ul este extins, monitorizăm pentru scăderea de viteză
                if (!shotWasDetected) {
                    if (getShooterCurrentRPM() < (rpmBeforePush * (1.0 - SHOT_CONFIRM_DIP_PERCENT))) {
                        shotWasDetected = true; // Am detectat aruncarea!
                    }
                }

                // Pasul 2: Așteptăm să treacă timpul alocat pentru împingere.
                if (outtakeTimer.milliseconds() > PUSH_TIME_MS) {
                    // Timpul a expirat, comandăm retragerea.
                    pusher.setPosition(RETRACT_POS);

                    // Pasul 3: AȘTEPTĂM AICI confirmarea de la senzorul magnetic.
                    // Trecem la starea următoare DOAR DUPĂ ce senzorul confirmă fizic retragerea.
                    if (isPusherRetracted()) {
                        // Pusher-ul este confirmat ca fiind retras. Putem continua în siguranță.
                        outtakeTimer.reset(); // Resetăm timer-ul pentru starea următoare
                        outtakeState = OuttakeState.CONFIRM_SHOT;
                    }
                    // Adăugăm și un timeout de siguranță, în caz că senzorul eșuează.
                    else if (outtakeTimer.milliseconds() > PUSH_TIME_MS + 2000) { // 2s siguranță
                        outtakeTimer.reset();
                        outtakeState = OuttakeState.CONFIRM_SHOT; // Forțăm trecerea pentru a nu bloca robotul
                    }
                }
                break;

            /**
             * STARE NOUĂ: CONFIRM_SHOT
             * Această stare se ocupă EXCLUSIV de logica de după ce pusher-ul s-a retras.
             */
            case CONFIRM_SHOT:
                // Așteptăm fereastra de timp pentru detecție.
                if (outtakeTimer.milliseconds() > RETRACT_TIME_MS + 200) {
                    // După ce a trecut fereastra, luăm o decizie finală.

                    if (shotWasDetected) {
                        // SUCCES: Am detectat lovitura, deci avansăm normal.
                        outtakePtr++;
                        if (outtakePtr >= outtakeOrder.length) {
                            outtakeState = OuttakeState.FINISHED;
                        } else {
                            occupied[outtakePtr] = false;
                            goToSlot(outtakeOrder[outtakePtr], true);
                            outtakeState = OuttakeState.ADVANCE;
                        }
                    } else {
                        // EȘEC: Nu am detectat lovitura. Verificăm dacă este prima sau a doua încercare.
                        if (shotRetryCounter < MAX_SHOT_RETRIES) {
                            // Este prima încercare eșuată. Incrementăm contorul și reîncercăm.
                            shotRetryCounter++;
                            triggerReady = true; // Reactivăm trigger-ul
                            outtakeState = OuttakeState.ALIGNING_FOR_SHOT; // Ne întoarcem la aliniere
                        } else {
                            // ESTE A DOUA ÎNCERCARE EȘUATĂ (sau am atins limita).
                            // Forțăm `shotWasDetected` pe true și avansăm, pretinzând că a fost un succes.
                            shotWasDetected = true;

                            outtakePtr++;
                            if (outtakePtr >= outtakeOrder.length) {
                                outtakeState = OuttakeState.FINISHED;
                            } else {
                                goToSlot(outtakeOrder[outtakePtr], true);
                                outtakeState = OuttakeState.ADVANCE;
                            }
                        }
                    }
                }
                break;

            case ADVANCE:
                // Așteptăm ca următorul slot să ajungă la poziție.
                // Când ajunge, reintrăm în ciclul de tragere.
                if (atTarget()) {
                    triggerReady = true; // Pre-armăm trigger-ul pentru tragere automată în lanț
                    outtakeState = OuttakeState.ALIGNING_FOR_SHOT;
                }
                break;

            case FINISHED:
                // Oprim shooter-ul și comandăm întoarcerea la poziția de start.
                setShooterTargetRPM(0.0);
                triggerReady = false;
                goToSlot(0, false); // Comanda de resetare este trimisă.

                // Trecem IMEDIAT la OUT_IDLE, fără a aștepta finalizarea mișcării.
                // Acest lucru permite autonomiei să continue.
                autoEnabled = true; // Permitem din nou funcționarea intake-ului automat
                outtakeState = OuttakeState.OUT_IDLE;
                break;
        }
    }

    public void skipTrow(){
        shotWasDetected = true;
    }

    private int[] buildFallbackOrder(OuttakePattern pattern) {
        List<Integer> result = new ArrayList<>();
        boolean[] used = new boolean[3];
        BallColor[] wanted;
        if (pattern == OuttakePattern.GPP) wanted = new BallColor[]{BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE};
        else if (pattern == OuttakePattern.PGP) wanted = new BallColor[]{BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE};
        else wanted = new BallColor[]{BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
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

    public boolean canChangeRPM(){
        return outtakeState != OuttakeState.OUT_IDLE;
    }

    public boolean canSkipShoot(){
        return triggerReady;
    }

    private BallColor detectBallColor() {
        NormalizedRGBA c1 = colorSensor1.getNormalizedColors();
        NormalizedRGBA c2 = colorSensor2.getNormalizedColors();
        float[] hsv1 = new float[3], hsv2 = new float[3];
        Color.colorToHSV(c1.toColor(), hsv1);
        Color.colorToHSV(c2.toColor(), hsv2);
        float hue = Math.max(hsv1[0], hsv2[0]);
        if (hue > 90 && hue < 185) return BallColor.GREEN;
        if (hue > 200 && hue < 320) return BallColor.PURPLE;
        return BallColor.UNKNOWN;
    }

    public boolean entrySlotHasBall() {
        // --- CONDIȚIA NOUĂ: Validăm citirea doar dacă suntem aliniați fizic ---
        // Dacă nu suntem la țintă (în toleranță), considerăm că nu vedem nicio bilă.
        if (!atTarget()) {
            timerRunning = false; // Resetăm timer-ul dacă ne mișcăm
            return false;
        }

        // --- Restul logicii rulează DOAR dacă suntem aliniați ---

        // Pasul 1: Citim distanța de la toți senzorii
        double mainDistance = entrySensor.getDistance(DistanceUnit.MM);
        double color1Distance = ((DistanceSensor) colorSensor1).getDistance(DistanceUnit.MM);
        double color2Distance = ((DistanceSensor) colorSensor2).getDistance(DistanceUnit.MM);

        // Pasul 2: Verificăm condiția "brută"
        boolean mainSeesBall = mainDistance < SLOT_OCCUPIED_MM;
        boolean color1SeesBall = color1Distance < COLOR_SENSOR_OCCUPIED_MM;
        boolean color2SeesBall = color2Distance < COLOR_SENSOR_OCCUPIED_MM;

        // Condiția brută este adevărată dacă unul dintre senzorii de culoare vede bila
        //boolean aSensorsSeeBall = color1SeesBall || color2SeesBall || mainSeesBall;
        boolean aSensorsSeeBall =  mainSeesBall;

        // Pasul 3: Aplicăm logica timer-ului de stabilitate
        if (aSensorsSeeBall) {
            // Dacă senzorii văd o bilă (și suntem aliniați), pornim timer-ul
            if (!timerRunning) {
                intakeTimer.reset();
                timerRunning = true;
            }
            // Returnăm 'true' doar dacă timer-ul a atins pragul de stabilitate
            return intakeTimer.milliseconds() >= SENSOR_DELAY_MS;
        } else {
            // Dacă senzorii nu văd bila (chiar dacă suntem aliniați), resetăm
            timerRunning = false;
            return false;
        }
    }

    public boolean isReadyToShoot() {
        return outtakeState == OuttakeState.ALIGNING_FOR_SHOT && atTarget();
    }
    public OuttakePattern getActivePattern() { return this.activePattern; }
    public boolean atTarget() {
        double currentVoltage = carouselFeedback.getVoltage();
        // Convertim voltajul citit (V) în milivolți (mV) pentru comparație
        double currentMilliVolts = currentVoltage * 1000.0;
        double error = Math.abs(currentMilliVolts - targetFeedbackMv);

        if (error <= FEEDBACK_TOLERANCE_MV) {
            // Dacă suntem în toleranță, verificăm dacă am stat suficient timp
            return atTargetTimer.milliseconds() >= AT_TARGET_STABILITY_MS;
        } else {
            // Dacă am ieșit din toleranță, resetăm cronometrul
            atTargetTimer.reset();
            return false;
        }
    }
    public double getFeedbackError() {
        double currentMilliVolts = carouselFeedback.getVoltage() * 1000.0;
        return currentMilliVolts - targetFeedbackMv;
    }

// --- De asemenea, adaugă aceste funcții ajutătoare pentru telemetrie ---

    public double getCurrentFeedbackMv() {
        return carouselFeedback.getVoltage() * 1000.0;
    }

    public double getTargetFeedbackMv() {
        return targetFeedbackMv;
    }

    public boolean allSlotsOccupied() { return occupied[0] && occupied[1] && occupied[2]; }
    public String getIntakeState() { return intakeState.name(); }
    public String getOuttakeState() { return outtakeState.name(); }
    public int getLogicalIndex() { return logicalIndex; }
    public OuttakeState getOuttakeStateEnum() {
        return this.outtakeState;
    }


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


    public double getMainDistance() {
        return entrySensor.getDistance(DistanceUnit.MM);
    }

    public double getColor1Distance() {
        return ((DistanceSensor) colorSensor1).getDistance(DistanceUnit.MM);
    }

    public double getColor2Distance() {
        return ((DistanceSensor) colorSensor2).getDistance(DistanceUnit.MM);
    }

    public double getCurrentVoltage() {
        return (batteryVoltageSensor.getVoltage());
    }


    @Override
    public void periodic() {
                // --- BUCLA DE CONTROL PENTRU SHOOTER (PIDF Manual) ---
        shooterController.setPID(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD);
        double currentShooterVelo = shooterMotor.getVelocity(); // În ticks/sec
        double targetShooterVelo = rpmToTicksPerSecond(currentTargetRPM);
        double pidCorrection = shooterController.calculate(currentShooterVelo,targetShooterVelo);
        double feedforward = targetShooterVelo * SHOOTER_kF;
        double shooterPower = feedforward + pidCorrection;
        shooterMotor.setPower(shooterPower);



        // Mașinile de stări
        if ((outtakeState == OuttakeState.OUT_IDLE) && autoEnabled) handleIntake();
        handleOuttake();
        // Acest bloc va rula acum DOAR dacă flag-ul 'isTeleOp' este activat.
        if (isTeleOp && allSlotsOccupied() && outtakeState == OuttakeState.OUT_IDLE && autoEnabled) {
            prepareOuttake(activePattern);
            intakeIsOn = false;
        }
    }

}
