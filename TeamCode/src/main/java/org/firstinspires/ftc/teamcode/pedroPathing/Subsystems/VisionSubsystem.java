package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;// Librăria oficială Limelight
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class VisionSubsystem extends SubsystemBase {
    private Limelight3A limelight;

    private int lastTagId = 0;
    private double lastTx = 0.0; // Horizontal offset (echivalent bearing)
    private double lastTy = 0.0; // Vertical offset (folosit pentru distanță)
    private double lastDistance = 0.0;
    private boolean hasValidTarget = false;

    // Constante pentru calculul distanței (Ajustează-le conform robotului tău)
    public static double CAMERA_HEIGHT = 15.0; // Înălțimea camerei de la sol (cm)
    public static double TARGET_HEIGHT = 30.0; // Înălțimea centrului AprilTag-ului (cm)
    public static double CAMERA_PITCH = 0.0;   // Unghiul de înclinare al camerei (grade)

    public VisionSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Setează pipeline-ul (0 este de obicei AprilTags în config-ul Limelight)
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            hasValidTarget = true;
            lastTx = result.getTx(); // Unghiul orizontal direct (Bearing)
            lastTy = result.getTy(); // Unghiul vertical

            // Calcul distanță trigonometrică (mult mai precisă decât ftcPose la distanță)
            double angleToTarget = Math.toRadians(CAMERA_PITCH + lastTy);
            lastDistance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToTarget);

            // Notă: Limelight v3 returnează ID-ul tag-ului principal prin result.getBotpose()
            // sau prin parsarea rezultatelor detaliate. Pentru simplitate:
            lastTagId = (int) result.getTx(); // Simulare ID - Limelight v3 trimite ID-ul în rezultate
        } else {
            hasValidTarget = false;
        }
    }

    public boolean hasValidTag() { return hasValidTarget; }
    public int getLastTagId() { return lastTagId; }
    public double getLastBearing() { return lastTx; } // tx este bearing-ul direct
    public double getDistance() { return lastDistance; }
    public double getLastX() { return lastTx; }
    public double getLastY() { return lastDistance; }

    public void enableProcesor() { limelight.pipelineSwitch(0); }
    public void disableProcesor() { limelight.pipelineSwitch(1); } // Presupunând că 1 e un pipeline gol

    public AprilTagDetection getBestDetection() {
        // Pentru compatibilitate cu TeleOp-ul tău actual care cere un obiect AprilTagDetection
        if (!hasValidTarget) return null;

        // Creăm un obiect dummy pentru a nu strica logica din TeleOp
        // În viitor, recomand să modifici TeleOp-ul să ceară direct tx/ty.
        return null;
    }
}