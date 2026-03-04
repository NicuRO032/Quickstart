package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.TeleOpCarousel1.intakeIsOn;

import com.seattlesolvers.solverslib.controller.PIDController;


@Config
public class CarouselSubsystem1 extends SubsystemBase {
    /* ================= CONSTANTE CARUSEL SERVO ================= */
    // Vectori pentru pozițiile de Intake (servo) - Rămân la fel
    private static final double[] INTAKE_POSITIONS = {0.476, 0.732, 0.98};
    private static final double[] INTAKE_FEEDBACK_MV = {1580.0, 2315.0, 3015.0};

    // Vectori pentru pozițiile de START ale fiecărei salve. Corespund sloturilor 0, 1, 2
    public static final double[] SALVO_START_POSITIONS   = {0.98, 0.98,  0.98};
    public static final double[] SALVO_START_FEEDBACK_MV = {3015.0, 3015.0, 3015.0};

    // Vectori pentru pozițiile de FINAL ale fiecărei salve.
    public static final double[] SALVO_END_POSITIONS     = {0.112,  0.112,  0.112};
    public static final double[] SALVO_END_FEEDBACK_MV   = {450.0, 450.0, 450.0};

    //Constante PENTRU SALVA LENTĂ, DE PRECIZIE
    public static final double[] SLOW_SALVO_PAUSE1_POS       = {0.65, 0.65, 0.65};
    public static final double[] SLOW_SALVO_PAUSE1_FEEDBACK  = {2072.0, 2072.0, 2072.0};
    public static final double[] SLOW_SALVO_PAUSE2_POS       = {0.392, 0.392, 0.392};
    public static final double[] SLOW_SALVO_PAUSE2_FEEDBACK  = {1345.0, 1345.0, 1345.0};
    public static int SLOW_SHOOT_PAUSE_MS = 500; // Pauza în milisecunde pentru recuperarea turației


    // Toleranța pentru atTarget
    public static double FEEDBACK_TOLERANCE_MV = 120;
    public static long AT_TARGET_STABILITY_MS = 20;
    public static double CAROUSEL_SERVO2_OFFSET = 0.02;


    // Constante shooter
    public static double SHOOTER_kP = 0.01;
    public static double SHOOTER_kI = 0.0;
    public static double SHOOTER_kD = 0.00001;
    public static double SHOOTER_kF = 0.00046;
    public static final double SHOOTER_MOTOR_CPR = 28.0;
    public static double DEFAULT_SHOOTER_RPM = 3200.0;

    /* ================= CONSTANTE ================= */

    public static final double SLOT_OCCUPIED_MM = 120.0;
    public static double COLOR_SENSOR_OCCUPIED_MM = 70.0;
    public static final long SENSOR_DELAY_MS = 10;



    /* ================= HARDWARE ================= */
    private final DcMotorEx shooterMotor1, shooterMotor2;
    private final Servo carouselServo1, carouselServo2;

    private final AnalogInput carouselFeedback;
    private final DistanceSensor entrySensor;
    private final NormalizedColorSensor colorSensor1, colorSensor2;
    private final IntakeSubsystem1 intake;

    /* ================= STATES ================= */
    public enum IntakeState {IDLE, STORE_AND_ADVANCE, MANUAL_MOVE, REVERSE_INTAKE}
    private IntakeState intakeState = IntakeState.IDLE;

    public enum OuttakeState { OUT_IDLE, PREPARING_SALVO, RELAXING_SERVO, SHOOTING_SALVO, FINISHED }
    private OuttakeState outtakeState = OuttakeState.OUT_IDLE;

    private enum SlowShootSequence { INACTIVE, STEP_1, PAUSE_1, STEP_2, PAUSE_2, STEP_3 }
    private SlowShootSequence slowShootState = SlowShootSequence.INACTIVE;

    /* ================= LOGIC ================= */

    private int logicalIndex = 0;
    private double targetServoPosition = 0.0;
    private double targetFeedbackMv = 0.0;
    private final ElapsedTime atTargetTimer = new ElapsedTime();
    private final ElapsedTime intakeReverseTimer = new ElapsedTime();
    private final PIDController shooterController;
    private double currentTargetRPM = 0.0;
    private final boolean[] occupied = new boolean[3];
    public static final BallColor[] slotColor = new BallColor[3];
    private boolean autoEnabled = true;
    private OuttakePattern activePattern = OuttakePattern.GPP;
    private int activeSalvoIndex = 0; // Indexul salvei (0, 1, 2) care se va executa
    private boolean needsAutoPrepare = false; //Flag pentru a cere pregătirea automată a outtake-ului

    public enum BallColor { GREEN, PURPLE, UNKNOWN }
    public enum OuttakePattern { GPP, PGP, PPG }

    private final ElapsedTime intakeTimer = new ElapsedTime();
    private final ElapsedTime outtakeTimer = new ElapsedTime();

    private boolean timerRunning = false;

    final float[] hsvValues1 = new float[3];
    final float[] hsvValues2 = new float[3];
    public boolean isTeleOp = false;

    /* ================= LOGIC PENTRU MIȘCARE LENTĂ SERVO ================= */
    private enum SlowMoveState {
        INACTIVE,
        MOVING
    }
    private SlowMoveState slowMoveState = SlowMoveState.INACTIVE;

    private double slowMoveStartPos;
    private double slowMoveTargetPos;
    private final ElapsedTime slowMoveTimer = new ElapsedTime();
    public static double SLOW_MOVE_DURATION_MS = 750; // Durata în milisecunde pentru mișcarea lentă.
    public static double STABILIZATION_DURATION_MS = 100; // Durata în milisecunde pentru stabilizare parghie.

    public CarouselSubsystem1(HardwareMap hardwareMap, IntakeSubsystem1 intake) {
        this.intake = intake;

        carouselServo1 = hardwareMap.get(Servo.class, "carouselServo1");
        carouselServo2 = hardwareMap.get(Servo.class, "carouselServo2");
        carouselFeedback = hardwareMap.get(AnalogInput.class, "axonFeedback");

        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "motorShooter1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "motorShooter2");
        entrySensor = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color1");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color2");

        shooterMotor1.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor2.setDirection(DcMotorEx.Direction.FORWARD);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterController = new PIDController(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD);

        for (int i = 0; i < 3; i++) {
            occupied[i] = false;
            slotColor[i] = BallColor.UNKNOWN;
        }

        resetForStart();
    }

    /* =================================================================================
     * MODIFICARE 4: FUNCȚII HELPER NOI PENTRU LOGICA INTELIGENTĂ
     * ================================================================================= */
    /**
     * Verifică dacă în carusel există combinația corectă de bile (1 Verde, 2 Mov).
     * @return true dacă mixul de bile este corect.
     */
    private boolean hasCorrectBallMix() {
        int greenCount = 0;
        int purpleCount = 0;
        for (BallColor color : slotColor) {
            if (color == BallColor.GREEN) {
                greenCount++;
            } else if (color == BallColor.PURPLE) {
                purpleCount++;
            }
        }
        return greenCount == 1 && purpleCount == 2;
    }

    /**
     * Determină ce salvă (0, 1, sau 2) produce ordinea de aruncare corectă
     * pentru a se potrivi cu pattern-ul cerut, dacă bilele permit.
     * @param pattern Pattern-ul de outtake dorit.
     * @return Indexul salvei (0, 1, sau 2) care realizează pattern-ul.
     */
    private int findCorrectSalvoForPattern(OuttakePattern pattern) {
        // Ordinea de ejectare pentru fiecare salvă:
        // - Salva 0 (start 0): ejectează în ordinea 0, 2, 1
        // - Salva 1 (start 1): ejectează în ordinea 1, 0, 2
        // - Salva 2 (start 2): ejectează în ordinea 2, 1, 0

        switch (pattern) {
            case GPP: // Verde trebuie să fie prima bilă ejectată
                if (slotColor[0] == BallColor.GREEN) return 0; // Salva 0 ejectează slotul 0 primul
                if (slotColor[1] == BallColor.GREEN) return 1; // Salva 1 ejectează slotul 1 primul
                if (slotColor[2] == BallColor.GREEN) return 2; // Salva 2 ejectează slotul 2 primul
                break;
            case PGP: // Verde trebuie să fie a doua bilă ejectată
                if (slotColor[2] == BallColor.GREEN) return 0; // Salva 0 ejectează slotul 2 al doilea
                if (slotColor[0] == BallColor.GREEN) return 1; // Salva 1 ejectează slotul 0 al doilea
                if (slotColor[1] == BallColor.GREEN) return 2; // Salva 2 ejectează slotul 1 al doilea
                break;
            case PPG: // Verde trebuie să fie a treia bilă ejectată
                if (slotColor[1] == BallColor.GREEN) return 0; // Salva 0 ejectează slotul 1 al treilea
                if (slotColor[2] == BallColor.GREEN) return 1; // Salva 1 ejectează slotul 2 al treilea
                if (slotColor[0] == BallColor.GREEN) return 2; // Salva 2 ejectează slotul 0 al treilea
                break;
        }
        // Fallback în caz că logica nu găsește o potrivire (nu ar trebui să se întâmple)
        return 0;
    }




    /**
     * Pregătește outtake-ul. Ia decizia inteligentă aici.
     */
    public void prepareOuttake(OuttakePattern pattern) {
        if (outtakeState != OuttakeState.OUT_IDLE) return;
        autoEnabled = false;
        intakeIsOn = false;

        int salvoToExecute;

        if (hasCorrectBallMix()) {
            // CAZ IDEAL: Avem bilele potrivite. Alegem salva care face pattern-ul.
            salvoToExecute = findCorrectSalvoForPattern(pattern);
        } else {
            // CAZ DE AVARIE: Mix de bile greșit. Alegem o salvă de default (0).
            salvoToExecute = 0;
        }

        this.activeSalvoIndex = salvoToExecute;
        setShooterTargetRPM(DEFAULT_SHOOTER_RPM);

        // Comandăm caruselul la poziția de START a salvei alese
        goToServoPosition(SALVO_START_POSITIONS[activeSalvoIndex], SALVO_START_FEEDBACK_MV[activeSalvoIndex]);
        outtakeState = OuttakeState.PREPARING_SALVO;
    }

    /**
     * Declanșează salva, dacă sistemul este pregătit.
     */

    public void triggerShoot() {
        // Folosim direct funcția helper, care acum are logica corectă.
        // Dacă nu suntem gata, ieșim.
        if (!isReadyToShoot()) {
            return;
        }

        // Comandăm mișcarea la poziția de FINAL a salvei stocate
        goToServoPosition(SALVO_END_POSITIONS[activeSalvoIndex], SALVO_END_FEEDBACK_MV[activeSalvoIndex]);
        outtakeState = OuttakeState.SHOOTING_SALVO;
    }

    public void triggerSlowShoot() {
        // Verificăm dacă suntem pregătiți ȘI dacă nu este deja o altă acțiune în curs
        if (!isReadyToShoot())  {
            return;
        }

        // Pornim secvența lentă, trecând în prima sa stare
        goToServoPosition(SLOW_SALVO_PAUSE1_POS[activeSalvoIndex], SLOW_SALVO_PAUSE1_FEEDBACK[activeSalvoIndex]);
        slowShootState = SlowShootSequence.STEP_1;
        //outtakeState = OuttakeState.SHOOTING_SALVO;
    }

    /**
     * Oprește totul și resetează sistemul.
     */
    public void abortAll() {
        setShooterTargetRPM(0.0);
        autoEnabled = true;
        for (int i = 0; i < 3; i++) {
            occupied[i] = false;
            slotColor[i] = BallColor.UNKNOWN;
        }

        resetForStart();
    }

    /**
     * Inițiază o mișcare lentă a caruselului de la poziția curentă la o țintă nouă.
     * @param targetPosition Ținta finală a servoului.
     */
    private void startSlowMove(double targetPosition) {
        // Citim poziția curentă a servoului pentru a ști de unde plecăm.
        // Folosim targetServoPosition ca o aproximație bună a poziției curente.
        this.slowMoveStartPos = this.targetServoPosition;
        this.slowMoveTargetPos = targetPosition;
        this.slowMoveTimer.reset();
        this.slowMoveState = SlowMoveState.MOVING;
    }



    /**
     * Execută un pas al mișcării lente. Această metodă trebuie apelată continuu în periodic().
     * Calculează și comandă o nouă poziție intermediară a servoului în fiecare ciclu.
     */
    private void handleSlowMove() {
        if (slowMoveState != SlowMoveState.MOVING) {
            return; // Nu facem nimic dacă nu suntem în mișcare lentă.
        }

        double elapsed = slowMoveTimer.milliseconds();
        // Calculăm progresul mișcării ca un procent (0.0 la 1.0)
        double progress = Math.min(elapsed / SLOW_MOVE_DURATION_MS, 1.0);

        // Interpolare liniară: calculăm poziția curentă pe baza progresului.
        double newPosition = slowMoveStartPos + (slowMoveTargetPos - slowMoveStartPos) * progress;

        // Comandăm servoului să meargă la această nouă poziție intermediară.
        // Folosim o valoare generică pentru feedback, deoarece ținta se schimbă constant.
        goToServoPosition(newPosition, (slowMoveStartPos + slowMoveTargetPos)/2 * 3000); // Feedback mediu

        // Dacă am ajuns la final (progres >= 1.0), oprim mișcarea lentă.
        if (progress >= 1.0) {

            double feedbackReduction = 220; // O valoare rotundă, sigură. Poate fi ajustată.
            double finalTargetFeedback = SALVO_START_FEEDBACK_MV[activeSalvoIndex] - feedbackReduction;

            // Comandăm poziția finală ȘI setăm ținta de feedback corectă.
            goToServoPosition(slowMoveTargetPos, finalTargetFeedback);

            slowMoveState = SlowMoveState.INACTIVE;
        }
    }

    // Metoda goToSlot devine privată și este înlocuită de goToServoPosition pentru claritate
    private void goToServoPosition(double position, double feedbackMv) {
        targetServoPosition = position;
        targetFeedbackMv = feedbackMv;
        carouselServo1.setPosition(targetServoPosition);
        carouselServo2.setPosition(targetServoPosition + CAROUSEL_SERVO2_OFFSET);
        atTargetTimer.reset();
    }

    // goToSlot este păstrată pentru uz intern, în special pentru Intake
    private void goToSlot(int targetSlot) {
        if (targetSlot < 0 || targetSlot > 2) return;
        logicalIndex = targetSlot;
        goToServoPosition(INTAKE_POSITIONS[targetSlot], INTAKE_FEEDBACK_MV[targetSlot]);
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
    public double getShooterCurrentRPM() { return ticksPerSecondToRpm(shooterMotor1.getVelocity()); }
    public double getShooterPower() { return shooterMotor1.getPower(); }



    public void resetForStart() {
        goToSlot(0);
        this.intakeState = IntakeState.IDLE;
        this.outtakeState = OuttakeState.OUT_IDLE;
        slowShootState = SlowShootSequence.INACTIVE;
        this.autoEnabled = true;
    }

public boolean isReadyToShoot() {
        return (outtakeState == OuttakeState.FINISHED || outtakeState == OuttakeState.OUT_IDLE) && atTarget() && isShooterReady();
}



    public void activateIntake() { autoEnabled = true; }


    public void forcePreload(BallColor s0, BallColor s1, BallColor s2) {
        occupied[0] = true; slotColor[0] = s0;
        occupied[1] = true; slotColor[1] = s1;
        occupied[2] = true; slotColor[2] = s2;
    }

    public void setActivePattern(OuttakePattern pattern) { this.activePattern = pattern; }


    public void manualStepLeft() {
        autoEnabled = false;
        int nextSlot = (logicalIndex - 1 + 3) % 3;
        goToSlot(nextSlot);
        intakeState = IntakeState.MANUAL_MOVE;
    }

    public void manualStepRight() {////////////Aici trebuie refacut true-->false si logivalIndex+1
        autoEnabled = false;
        int nextSlot = (logicalIndex +1 ) % 3;
        goToSlot(nextSlot);
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
                    // Caruselul s-a umplut.
                    autoEnabled = false; // Oprește ciclul automat
                    intakeIsOn = false; // Setează flag-ul pentru TeleOp
                    needsAutoPrepare = true;

                    // Comandă direct inversarea motorului
                    intake.setPower(0.7); // Putere pozitivă pentru a scoate bila
                    intakeReverseTimer.reset();
                    intakeState = IntakeState.REVERSE_INTAKE;

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
                        goToSlot(nextEmptySlot);
                    }

                    // IMPORTANT: După ce am comandat mișcarea, ne întoarcem IMEDIAT la IDLE.
                    // Mașina de stări de intake și-a terminat treaba pentru această bilă.
                    // Acum așteaptă dispariția bilei curente și apariția uneia noi.
                    intakeState = IntakeState.IDLE;
                }
                break;

            //reversing the intake after we load all balls
            case REVERSE_INTAKE:
                // Așteptăm să treacă timpul de inversare
                if (intakeReverseTimer.milliseconds() > 400) {
                    intake.stop(); // Oprim motorul de intake
                    intakeState = IntakeState.IDLE; // Revenim la starea de așteptare
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

    // în CarouselSubsystem1.java, înlocuiește handleOuttake()

    private void handleOuttake() {// Prioritizăm mișcarea lentă. Dacă una e în curs, nu executăm altceva.
        if (slowMoveState == SlowMoveState.MOVING) {
            return;
        }

        switch (outtakeState) {
            case OUT_IDLE:
                // Nu facem nimic, așteptăm comenzi
                break;

            case PREPARING_SALVO:
                // STAREA 1: Așteptăm ca servoul să ajungă la poziția de start a salvei.
                if (atTarget()) {
                    // A ajuns! Acum pornim cronometrul.
                    outtakeTimer.reset();
                    // Trecem în starea de așteptare/relaxare.
                    outtakeState = OuttakeState.RELAXING_SERVO;
                }
                break;

            case RELAXING_SERVO:
                // STAREA 2: Am ajuns la țintă și cronometrul a pornit.
                if (outtakeTimer.milliseconds() > STABILIZATION_DURATION_MS) {
                    // A trecut timpul de stabilizare, Pornim MIȘCAREA LENTĂ de relaxare.
                    double currentTarget = targetServoPosition;
                    double newTarget = currentTarget - 0.07; // Micșorăm poziția

                    // Comandăm MIȘCAREA LENTĂ în loc de cea instantanee
                    startSlowMove(newTarget);

                    // Trecem într-o stare finală de "gata de tragere".
                    outtakeState = OuttakeState.FINISHED;
                }
                break;

            case FINISHED:
                // STAREA 3: Sistemul este acum în poziția finală, relaxată, gata de tragere.
                // Așteptăm ca mișcarea lentă să se termine (slowMoveState devine INACTIVE)
                // și apoi ca atTarget() să fie adevărat pentru noua poziție.
                break;

            case SHOOTING_SALVO:
                // Am comandat mișcarea de tragere. Acum așteptăm să ajungă la final.
                if (atTarget()) {
                    // Salva s-a terminat. Resetăm totul.
                    abortAll();
                }
                break;
        }
    }

    /**
     * Gestionează secvența de aruncare lentă, bazată pe feedback și un timer.
     * Rulează independent de mașina de stări principală a outtake-ului.
     */
    private void handleSlowShoot() {
        if (slowShootState == SlowShootSequence.INACTIVE) return;

        switch (slowShootState) {
            case STEP_1:
                // Nu dăm comandă aici (a fost dată în triggerSlowShoot)
                if (atTarget()) {
                    outtakeTimer.reset();
                    slowShootState = SlowShootSequence.PAUSE_1;
                }
                break;

            case PAUSE_1:
                if (outtakeTimer.milliseconds() >= SLOW_SHOOT_PAUSE_MS) {
                    // COMANDĂM AICI (o singură dată, înainte să plecăm din stare)
                    goToServoPosition(SLOW_SALVO_PAUSE2_POS[activeSalvoIndex], SLOW_SALVO_PAUSE2_FEEDBACK[activeSalvoIndex]);
                    slowShootState = SlowShootSequence.STEP_2;
                }
                break;

            case STEP_2:
                // DOAR AȘTEPTĂM. Nu punem goToServoPosition aici!
                if (atTarget()) {
                    outtakeTimer.reset();
                    slowShootState = SlowShootSequence.PAUSE_2;
                }
                break;

            case PAUSE_2:
                if (outtakeTimer.milliseconds() >= SLOW_SHOOT_PAUSE_MS) {
                    // COMANDĂM POZIȚIA FINALĂ AICI
                    goToServoPosition(SALVO_END_POSITIONS[activeSalvoIndex], SALVO_END_FEEDBACK_MV[activeSalvoIndex]);
                    slowShootState = SlowShootSequence.STEP_3;
                }
                break;

            case STEP_3:
                // Așteptăm confirmarea finală
                if (atTarget()) {
                    //outtakeState = OuttakeState.SHOOTING_SALVO; // Opțional, poți pune direct FINISHED
                    //slowShootState = SlowShootSequence.INACTIVE;
                    abortAll();
                }
                break;
        }
    }

    public boolean canChangeRPM(){
        return outtakeState != OuttakeState.OUT_IDLE;
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

    public boolean isShooterReady() { return Math.abs(getShooterCurrentRPM() - currentTargetRPM) < 500; }

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

    public IntakeState getIntakeStateEnum() {
        return this.intakeState;
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
    public SlowShootSequence getSlowShootState() {
        return this.slowShootState;
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



    @Override
    public void periodic() {
                // --- BUCLA DE CONTROL PENTRU SHOOTER (PIDF Manual) ---
        shooterController.setPID(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD);
        double currentShooterVelo = shooterMotor1.getVelocity(); // În ticks/sec
        double targetShooterVelo = rpmToTicksPerSecond(currentTargetRPM);
        double pidCorrection = shooterController.calculate(currentShooterVelo,targetShooterVelo);
        double feedforward = targetShooterVelo * SHOOTER_kF;
        double shooterPower = feedforward + pidCorrection;
        shooterMotor1.setPower(shooterPower);
        shooterMotor2.setPower(shooterPower);

        handleSlowMove();//asezare parghie shooter
        handleSlowShoot();//salva lenta

        // Mașinile de stări
        if (outtakeState == OuttakeState.OUT_IDLE) {
            handleIntake();
        }
        handleOuttake();
        // Acest bloc va rula acum DOAR dacă flag-ul 'isTeleOp' este activat.
        if (isTeleOp && allSlotsOccupied() && outtakeState == OuttakeState.OUT_IDLE && autoEnabled) {
            prepareOuttake(activePattern);
            intakeIsOn = false;
        }

        if (needsAutoPrepare) {
            // O executăm DOAR dacă ambele FSM-uri sunt în repaus (IDLE).
            // Asta garantează că ciclul de REVERSE_INTAKE s-a terminat.
            if (intakeState == IntakeState.IDLE && outtakeState == OuttakeState.OUT_IDLE) {
                prepareOuttake(activePattern);
                needsAutoPrepare = false; // Resetăm flag-ul după ce am pornit pregătirea
            }
        }
    }

}
