package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.TeleOpCarousel1.intakeIsOn;

import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;

@Config
public class CarouselSubsystem1 extends SubsystemBase {
    // PIDF pentru motorul caruselului
    // Variabile pentru PID-ul nostru manual
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private int currentError = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();

// --- COEFICIENȚI PENTRU MIȘCĂRI MARI (eroare > 0.8 sloturi) ---
    public static double kP_COARSE = 0.0001; // kP mai mic pentru a preveni oscilațiile
    public static double kD_COARSE = 0.00001;     // Oprește kD când suntem departe

    // --- COEFICIENȚI PENTRU MIȘCĂRI FINE (eroare < 0.8 sloturi) ---
    public static double kP_FINE = 0.0004; // kP mai mare pentru precizie (similar cu ce aveai)
    public static double kD_FINE = 0.00002; // kD pentru a opri overshoot-ul la final

    // kI și kF sunt refolosiți
    public static double kI = 0.0;
    public static double kF = 0.0;

    // shooter
    public static double SHOOTER_kP = 0.001;
    public static double SHOOTER_kI = 0.0;
    public static double SHOOTER_kD = 0.000001;
    public static double SHOOTER_kF = 0.00045;

    public static final double SHOOTER_MOTOR_CPR = 28.0;
    public static double DEFAULT_SHOOTER_RPM = 2000.0;
    public static double SHOT_CONFIRM_DIP_PERCENT = 0.05; // Acum se aplică la RPM
    private double rpmBeforePush = 0.0;
    private boolean shotWasDetected = false;
    private final PIDController shooterController;
    //private final PIDController carouselController;
    private double currentTargetRPM = 0.0; // Ținta pentru shooter, în RPM




    /* ================= CONSTANTE ================= */
    public static final float TICKS_PER_SLOT = 8192/3f;
    public static final float OUTTAKE_OFFSET_SLOTS = 1.5f;
    public static double POWER_CAROUSEL = 0.8;
    public static int POSITION_TOLERANCE = 100;
    public static long AT_TARGET_STABILITY_MS = 75; // Timpul de stabilitate

    public static final double SLOT_OCCUPIED_MM = 100.0;
    public static double COLOR_SENSOR_OCCUPIED_MM = 70.0;
    public static final long SENSOR_DELAY_MS = 25;
    public static final double PUSH_POS = 0.1;
    public static final double RETRACT_POS = 0.5;
    public static long PUSH_TIME_MS = 400;
    public static long RETRACT_TIME_MS = 200;
    public static final double JOG_ON_POS = 0.25;
    public static final double JOG_OFF_POS = 0.0;

    /* ================= HARDWARE ================= */
    private final DcMotorEx motorCarousel, shooterMotor;
    private final DcMotorEx encoderCarusel; // <--ENCODERUL EXTERN
    private final DistanceSensor entrySensor;
    private final NormalizedColorSensor colorSensor1, colorSensor2;
    private final Servo pusher;
    private final Servo jogServo;

    /* ================= STATES ================= */
    public enum IntakeState {IDLE, STORE_AND_ADVANCE, MANUAL_MOVE}
    private IntakeState intakeState = IntakeState.IDLE;

    public enum OuttakeState { OUT_IDLE, PREPARE_READY, ALIGNING_FOR_SHOT, PUSH, CONFIRM_SHOT, WAIT_RETRACT, ADVANCE, FINISHED }
    private OuttakeState outtakeState = OuttakeState.OUT_IDLE;

    /* ================= LOGIC ================= */
    private int globalIndex = 0; // Pentru telemetrie
    private int logicalIndex = 0;
    private int targetPosition = 0;
    private final ElapsedTime atTargetTimer = new ElapsedTime(); //Cronometru pentru atTarget

    private final boolean[] occupied = new boolean[3];
    public static final BallColor[] slotColor = new BallColor[3];
    private boolean autoEnabled = true, ballHandled = false;

    private int[] outtakeOrder = new int[0];
    private int outtakePtr = 0;
    private boolean triggerReady = false;

    private boolean isCarouselPidActive = true;
    private double currentTargetVelocity = 0.0;

    public enum BallColor { GREEN, PURPLE, UNKNOWN }
    public enum OuttakePattern { GPP, PGP, PPG }
    private OuttakePattern activePattern = OuttakePattern.GPP;

    private final ElapsedTime intakeTimer = new ElapsedTime();
    private final ElapsedTime outtakeTimer = new ElapsedTime();
    private boolean timerRunning = false;

    final float[] hsvValues1 = new float[3];
    final float[] hsvValues2 = new float[3];

    public CarouselSubsystem1(HardwareMap hardwareMap) {


        motorCarousel = hardwareMap.get(DcMotorEx.class, "motorCarusel");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "motorShooter");
        entrySensor = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color1");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color2");
        pusher = hardwareMap.get(Servo.class, "pusher");
        jogServo = hardwareMap.get(Servo.class, "jogServo");
        encoderCarusel = hardwareMap.get(DcMotorEx.class, "encoderCarusel"); // Folosim DcMotorEx pentru a citi encoderul

        encoderCarusel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderCarusel.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterController = new PIDController(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD);

        motorCarousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorCarousel.setPower(0);
        motorCarousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //carouselController = new PIDController(kP, kI, kD);

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
    public double getShooterTargetRPM() { return this.currentTargetRPM; }
    public double getShooterCurrentRPM() { return ticksPerSecondToRpm(shooterMotor.getVelocity()); }
    public double getRpmBeforePush() { return rpmBeforePush; }


    private void goToSlot(int targetSlot, boolean isOuttake) {
        int currentLogicalAt12 = (globalIndex % 3 + 3) % 3;
        int diff = targetSlot - currentLogicalAt12;
        if (diff > 1) diff -= 3;
        else if (diff < -1) diff += 3;
        globalIndex += diff;
        float totalSlotsToMove = (float)globalIndex;
        if (isOuttake) {
            totalSlotsToMove -= OUTTAKE_OFFSET_SLOTS;
        }
        targetPosition = Math.round(totalSlotsToMove * TICKS_PER_SLOT);
        logicalIndex = targetSlot;
        integralSum = 0.0;
        lastError = 0.0;
        pidTimer.reset();
    }

    public void resetForStart() {
        motorCarousel.setPower(0);
        encoderCarusel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.targetPosition = 0;
        this.globalIndex = 0;
        this.logicalIndex = 0;
        this.intakeState = IntakeState.IDLE;
        this.outtakeState = OuttakeState.OUT_IDLE;
        this.autoEnabled = false;
        this.ballHandled = false;
        this.triggerReady = false;
    }

    public void jogCarousel(double power) {
        // 1. Dezactivează bucla PID din periodic()
        this.isCarouselPidActive = false;

        // 2. Aplică direct puterea la motor.
        // Motorul este deja în RUN_WITHOUT_ENCODER, deci nu trebuie schimbat modul.
        motorCarousel.setPower(power);
    }

    public void jogServoPos(double pos){
        jogServo.setPosition(pos);
    }

    public void confirmAlignment() {
        // 1. Oprim orice putere manuală rămasă de la jog.
        motorCarousel.setPower(0);

        // 2. RESETĂM ENCODERUL EXTERN - stabilește noul zero fizic.
        encoderCarusel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // 3. Actualizăm variabilele software pentru a reflecta noul zero.
        this.targetPosition = 0;
        this.logicalIndex = 0;
        this.globalIndex = 0;

        // 4. REACTIVĂM PID-ul. La următoarea rulare a lui periodic(),
        // PID-ul va vedea target=0, actual=0 și va menține ferm poziția.
        integralSum = 0.0;
        lastError = 0.0;
        pidTimer.reset();
        this.isCarouselPidActive = true;

        // Resetăm și stările mașinilor de stări.
        this.intakeState = IntakeState.IDLE;
        this.outtakeState = OuttakeState.OUT_IDLE;
        this.triggerReady = false;
        this.autoEnabled = true;
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
        switch (outtakeState) {
            // STARE NOUĂ: Doar comandă mișcarea și trece mai departe
            case PREPARE_READY:
                // 1. Comandăm mișcarea caruselului (acest apel resetează timer-ul și erorile PID)
                goToSlot(outtakeOrder[outtakePtr], true);

                // 2. Trecem IMEDIAT la o stare dedicată de așteptare.
                // Astfel, în următoarea buclă, PID-ul va porni corect, cu un 'dt' normal.
                outtakeState = OuttakeState.ALIGNING_FOR_SHOT;
                break;

            // STARE NOUĂ: Așteaptă alinierea și comanda de la pilot
            case ALIGNING_FOR_SHOT:
                // Așteptăm ca și caruselul să ajungă la țintă ȘI pilotul să apese pe trăgaci
                if (atTarget() && triggerReady) {
                    // Când ambele condiții sunt îndeplinite, suntem gata de aruncare
                    rpmBeforePush = getShooterCurrentRPM(); // Salvăm RPM-ul exact înainte de a împinge
                    shotWasDetected = false; // Resetăm flag-ul de detecție
                    pusher.setPosition(PUSH_POS); // Împingem bila
                    outtakeTimer.reset();
                    outtakeState = OuttakeState.PUSH;
                }
                break;

            case PUSH:
                // În timp ce pusher-ul este extins, monitorizăm pentru scăderea de viteză
                if (!shotWasDetected) {
                    boolean dipOccurred = getShooterCurrentRPM() < (rpmBeforePush * (1.0 - SHOT_CONFIRM_DIP_PERCENT));
                    if (dipOccurred) {
                        shotWasDetected = true; // Am detectat aruncarea!
                    }
                }

                // Așteptăm ca pusher-ul să-și termine cursa de împingere
                if (outtakeTimer.milliseconds() > PUSH_TIME_MS) {
                    pusher.setPosition(RETRACT_POS); // Începem retragerea
                    outtakeTimer.reset();
                    outtakeState = OuttakeState.WAIT_RETRACT;
                }
                break;

            case WAIT_RETRACT:
                // Așteptăm retragerea completă a pusher-ului
                if (outtakeTimer.milliseconds() > RETRACT_TIME_MS) {
                    if (shotWasDetected) {
                        // SUCCES! Aruncarea a fost detectată.
                        // Marcăm slotul ca gol și avansăm la următoarea bilă.
                        occupied[outtakeOrder[outtakePtr]] = false;
                        slotColor[outtakeOrder[outtakePtr]] = BallColor.UNKNOWN;
                        outtakePtr++; // Trecem la următoarea bilă din secvență

                        if (outtakePtr >= outtakeOrder.length) {
                            // Am terminat toate bilele, încheiem secvența
                            outtakeState = OuttakeState.FINISHED;
                        } else {
                            // Altfel, avansăm caruselul la următorul slot
                            goToSlot(outtakeOrder[outtakePtr], true);
                            outtakeState = OuttakeState.ADVANCE;
                        }
                    } else {
                        // EȘEC! Aruncarea NU a fost detectată.
                        // Nu avansăm pointer-ul (outtakePtr) și reîncercăm.
                        // Suntem deja aliniați, deci doar re-încercăm push-ul
                        rpmBeforePush = getShooterCurrentRPM();
                        shotWasDetected = false;
                        pusher.setPosition(PUSH_POS);
                        outtakeTimer.reset();
                        outtakeState = OuttakeState.PUSH;
                    }
                }
                break;

            case ADVANCE:
                // Așteptăm ca următorul slot să ajungă la poziție (pentru aruncările în lanț)
                if (atTarget()) {
                    rpmBeforePush = getShooterCurrentRPM(); // Salvăm viteza pentru aruncarea în lanț
                    shotWasDetected = false; // Resetăm flag-ul
                    pusher.setPosition(PUSH_POS);
                    outtakeTimer.reset();
                    // Ne întoarcem la starea PUSH pentru a lansa următoarea bilă
                    outtakeState = OuttakeState.PUSH;
                }
                break;

            case FINISHED:
                // Oprim shooter-ul și ne întoarcem la poziția de start
                setShooterTargetRPM(0.0);
                triggerReady = false;
                goToSlot(0, false); // Comandăm întoarcerea la slotul 0
                // Așteptăm să ajungă fizic la 0 înainte de a încheia complet
                if (Math.abs(targetPosition - getCurrentPosition()) < POSITION_TOLERANCE * 2) {
                    autoEnabled = true; // Permitem din nou funcționarea intake-ului automat
                    outtakeState = OuttakeState.OUT_IDLE;
                }
                break;
        }
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
        if (!isAligned()) {
            timerRunning = false; // Resetăm timer-ul dacă ne mișcăm
            return false;
        }

        // --- Restul logicii rulează DOAR dacă suntem aliniați ---

        // Pasul 1: Citim distanța de la toți senzorii
        double mainDistance = entrySensor.getDistance(DistanceUnit.MM);
        double color1Distance = ((DistanceSensor) colorSensor1).getDistance(DistanceUnit.MM);
        double color2Distance = ((DistanceSensor) colorSensor2).getDistance(DistanceUnit.MM);

        // Pasul 2: Verificăm condiția "brută"
        boolean color1SeesBall = color1Distance < COLOR_SENSOR_OCCUPIED_MM;
        boolean color2SeesBall = color2Distance < COLOR_SENSOR_OCCUPIED_MM;

        // Condiția brută este adevărată dacă unul dintre senzorii de culoare vede bila
        boolean allSensorsSeeBall = color1SeesBall || color2SeesBall;

        // Pasul 3: Aplicăm logica timer-ului de stabilitate
        if (allSensorsSeeBall) {
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

    public boolean isReadyToShoot() { return outtakeState == OuttakeState.PREPARE_READY && atTarget(); }
    public OuttakePattern getActivePattern() { return this.activePattern; }
    public boolean atTarget() {
        boolean isWithinTolerance = Math.abs(encoderCarusel.getCurrentPosition() - targetPosition) < POSITION_TOLERANCE;
        if (isWithinTolerance) {
            return atTargetTimer.milliseconds() >= AT_TARGET_STABILITY_MS;
        } else {
            atTargetTimer.reset();
            return false;
        }
    }

    private boolean isAligned() {
        return Math.abs(encoderCarusel.getCurrentPosition() - targetPosition) < POSITION_TOLERANCE;
    }
    public boolean allSlotsOccupied() { return occupied[0] && occupied[1] && occupied[2]; }
    public String getIntakeState() { return intakeState.name(); }
    public String getOuttakeState() { return outtakeState.name(); }
    public int getLogicalIndex() { return logicalIndex; }
    public int getGlobalIndex() { return globalIndex; }
    public int getTargetPosition() { return targetPosition; }
    public int getCurrentPosition() { return encoderCarusel.getCurrentPosition(); }
    public double getPIDError() { return this.currentError;}
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
    @Override
    public void periodic() {
                // --- BUCLA DE CONTROL PENTRU SHOOTER (PIDF Manual) ---
        //setShooterTargetRPM(DEFAULT_SHOOTER_RPM);//doar pentru tuning,va trebui eliminata
        shooterController.setPID(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD);
        double currentShooterVelo = shooterMotor.getVelocity(); // În ticks/sec
        double targetShooterVelo = rpmToTicksPerSecond(currentTargetRPM);
        double pidCorrection = shooterController.calculate(currentShooterVelo,targetShooterVelo);
        double feedforward = targetShooterVelo * SHOOTER_kF;
        double shooterPower = feedforward + pidCorrection;
        shooterMotor.setPower(shooterPower);

        // --- BUCLA DE CONTROL CU GAIN SCHEDULING (două seturi de coeficienți) ---
        if (isCarouselPidActive) {
            double dt = pidTimer.seconds();
            pidTimer.reset();

            int currentPosition = encoderCarusel.getCurrentPosition();
            int error = targetPosition - currentPosition;
            this.currentError = error;

            // --- MODIFICARE CHEIE: Selectarea dinamică a coeficienților ---
            double kP_actual;
            double kD_actual;

            // Când eroarea e mai mare de 1.1 dintr-un slot, folosim coeficienți slabi.
            if (Math.abs(error) > TICKS_PER_SLOT * 1.1) {
                kP_actual = kP_COARSE;
                kD_actual = kD_COARSE;
            } else {
                // Când suntem aproape, folosim coeficienții agresivi pentru blocare fermă.
                kP_actual = kP_FINE;
                kD_actual = kD_FINE;
            }
            // ----------------------------------------------------------------

            // Se calculează PID-ul folosind coeficienții aleși
            double p_term = kP_actual * error;
            integralSum += error * dt; // Chiar dacă kI e 0, lăsăm asta
            double i_term = kI * integralSum;


            double derivative = (dt > 0 && lastError != 0) ? (error - lastError) / dt : 0; // <-- ADAUGĂ `&& lastError != 0`
            double d_term = kD_actual * derivative;

            // --- Puterea totală ---
            double motorPower = p_term + i_term + d_term + kF;

            // Plafonarea finală a puterii
            motorPower = Math.max(-POWER_CAROUSEL, Math.min(motorPower, POWER_CAROUSEL));

            motorCarousel.setPower(motorPower);

            // Salvăm eroarea REALĂ pentru calculul derivatei
            lastError = error;
        }


        // Mașinile de stări
        if ((outtakeState == OuttakeState.OUT_IDLE) && autoEnabled) handleIntake();
        handleOuttake();
        if (allSlotsOccupied() && outtakeState == OuttakeState.OUT_IDLE && autoEnabled) {
            prepareOuttake(activePattern);
            intakeIsOn = false;
        }
    }

}
