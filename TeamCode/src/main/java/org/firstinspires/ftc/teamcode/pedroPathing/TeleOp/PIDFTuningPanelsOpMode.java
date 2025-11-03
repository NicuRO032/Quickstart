package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import com.seattlesolvers.solverslib.util.MathUtils;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.configurables.annotations.Configurable;

@TeleOp(name = "PIDF Tuning (Panels)", group = "Tuning")
@Configurable
public class PIDFTuningPanelsOpMode extends LinearOpMode {

    private DcMotorEx motor;
    private PIDFController pidfController;
    private ElapsedTime timer;

    // === Coeficienți PIDF inițiali ===
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    // === Parametri sistem ===
    public static double targetVelocity = 0.5;
    public static boolean motorReversed = false;

    // === Panou interactiv ===
    //private Panel panel;
    //private panelsTelemetry = PanelsTelemetry.telemetry;
    public static TelemetryManager telemetryM;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware setup
        //motor = hardwareMap.get(DcMotorEx.class, "motorFlywheel");
        Motor motor = new Motor(hardwareMap, "motorFlywheel", Motor.GoBILDA.RPM_312);
        //motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setRunMode(Motor.RunMode.VelocityControl);
        //motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        //motor.setDirection(motorReversed ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);

        pidfController = new PIDFController(kP, kI, kD, kF);
        timer = new ElapsedTime();

        // === Creare panou pentru reglaj PIDF ===
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();



        telemetryM.addLine("✅ PIDF Tuning (Panels) Ready");
        telemetryM.addLine("➡️ Accesează panoul 'PIDF Tuning' pentru reglaje live");

        //telemetryM.addData("kP", kP);
        //telemetryM.addData("kI", kI);
        //telemetryM.addData("kD", kD);
        //telemetryM.addData("kF", kF);
        //telemetryM.addData("Target Velocity", targetVelocity);

        //telemetryM.debug("kP" + kP);
        //telemetryM.debug("kI" + kI);
        //telemetryM.debug("kD" + kD);
        //telemetryM.debug("kF" + kF);
        //telemetryM.debug("Target Velocity" + targetVelocity);
        //telemetryM.update();

        waitForStart();

        //double lastTime = timer.seconds();
        //int lastPos = motor.getCurrentPosition();

        while (opModeIsActive()) {
            // === Actualizare valori din panou ===


            //kP = panel.get("kP");
            //kI = panel.get("kI");
            //kD = panel.get("kD");
            //kF = panel.get("kF");
            //targetVelocity = panel.get("Target Velocity");

            pidfController.setPIDF(kP, kI, kD, kF);

            //double now = timer.seconds();
            //double deltaTime = now - lastTime;
            //int currentPos = motor.getCurrentPosition();
            //double currentVelocity = (currentPos - lastPos) / deltaTime;

            // PIDF control
            //double error = pidfController.getVelocityError();
            //double correction = pidfController.update(targetVelocity, currentVelocity, deltaTime);
            //double power = MathUtils.clamp(error, -1.0, 1.0);

            double output = pidfController.calculate(
                    motor.get(), targetVelocity
            );
            double power=output;

            motor.set(power);

            // Telemetrie
            //telemetry.addData("Target Velocity", "%.1f", targetVelocity);
            //telemetry.addData("Current Velocity", "%.1f", currentVelocity);
            //telemetry.addData("Power", "%.2f", power);
            //telemetry.addData("kP", kP);
            //telemetry.addData("kI", kI);
            //telemetry.addData("kD", kD);
            //telemetry.addData("kF", kF);
            //telemetry.update();

            //telemetryM.debug("kP" + kP);
            //telemetryM.debug("kI" + kI);
            //telemetryM.debug("kD" + kD);
            //telemetryM.debug("kF" + kF);
            //telemetryM.debug("Target Velocity" + targetVelocity);
            //telemetryM.debug("Current Velocity" + motor.getVelocity());
            //telemetryM.debug("Current Power" + power);


            telemetryM.addData("Target Velocity", targetVelocity);
            telemetryM.addData("Current Velocity", motor.get());
            telemetryM.addData("Current Power" , power);

            telemetryM.update();

            // actualizare pentru pasul următor
            //lastPos = currentPos;
            //lastTime = now;

            //sleep(50);
        }
    }
}
