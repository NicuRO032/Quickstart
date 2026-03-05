package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;// Librăria oficială Limelight
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.opencv.core.Point;

@Config
public class VisionSubsystem extends SubsystemBase {
    private Limelight3A limelight;

    private int lastTagId = 0;
    private double lastTx = 0.0; // Horizontal offset (echivalent bearing)
    private double lastTy = 0.0; // Vertical offset (folosit pentru distanță)
    private double lastDistance = 0.0;
    private boolean hasValidTarget = false;

    // Constante pentru calculul distanței (Ajustează-le conform robotului tău)
    public static double CAMERA_HEIGHT = 30.0; // Înălțimea camerei de la sol (cm)
    public static double TARGET_HEIGHT = 110.0; // Înălțimea centrului AprilTag-ului (cm)
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

        // Verificăm dacă Limelight vede un AprilTag (Fiducial)
        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            hasValidTarget = true;

            // Luăm prima detecție din listă (cea mai relevantă)
            com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fr = result.getFiducialResults().get(0);

            lastTagId = (int) fr.getFiducialId(); // ID-ul real (20, 24 etc.)
            lastTx = fr.getTargetXDegrees();     // Unghiul orizontal (bearing)
            lastTy = fr.getTargetYDegrees();     // Unghiul vertical (pitch)

            // Calcul distanță
            double angleToTarget = Math.toRadians(CAMERA_PITCH + lastTy);
            lastDistance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToTarget);
        } else {
            hasValidTarget = false;
        }
    }

    // Metode simple pentru a citi datele din exterior
    public boolean hasValidTag() { return hasValidTarget; }
    public int getLastTagId() { return lastTagId; }
    public double getLastBearing() { return lastTx; }



    public double getDistance() { return lastDistance; }
    public double getLastX() { return lastTx; }
    public double getLastY() { return lastDistance; }

    //public void enableProcesor() { limelight.pipelineSwitch(8); }
    //public void disableProcesor() { limelight.pipelineSwitch(1); } // Presupunând că 1 e un pipeline gol


}