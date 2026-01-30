package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import android.util.Size;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;
@Config
public class VisionSubsystem extends SubsystemBase {
    public static int EXPOSURE_MS = 6;
    public static int GAIN = 100;
    public static double FOCUS = 0.0;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private int lastTagId = 0;
    private double lastBearing = 0.0;
    private boolean hasValidTag = false;

    public VisionSubsystem(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
        // ▼▼▼ ADAUGĂ ACEST APEL ▼▼▼
        // Așteptăm ca portalul să fie gata și apoi setăm controalele manuale
        // Valorile de start: expunere 6ms, gain 100, focus fixat la infinit (0.0)
        //setManualCameraControls(EXPOSURE_MS, GAIN, FOCUS);
    }

    // ▼▼▼ ADAUGĂ ACEASTĂ METODĂ NOUĂ COMPLETĂ ▼▼▼
    public boolean setManualCameraControls(int exposureMS, int gain, double focus) {
        // Asigură-te că portalul de viziune este deschis și camera face streaming
        if (visionPortal == null || visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            // Poți adăuga un telemetry.log.add("Aștept camera...") aici pentru debug
            return false;
        }

        // --- Setare Expunere și Gain ---
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);

        // --- Setare Focalizare ---
        FocusControl focusControl = visionPortal.getCameraControl(FocusControl.class);
        if (focusControl.getMode() != FocusControl.Mode.Fixed) {
            focusControl.setMode(FocusControl.Mode.Fixed);
        }
        focusControl.setFocusLength(focus); // Valoare între 0.0 (infinit) și 1.0 (apropiat)

        return true;
    }
    // ▲▲▲ SFÂRȘIT METODĂ NOUĂ ▲▲▲

    @Override
    public void periodic() {
        //setManualCameraControls(EXPOSURE_MS, GAIN, FOCUS);
        updateAprilTagData();
    }

    private void updateAprilTagData() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        hasValidTag = false;
        lastTagId = 0;
        lastBearing = 0.0;

        if (!detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);

            if (tag != null && tag.metadata != null) {
                lastTagId = tag.id;
                lastBearing = tag.ftcPose.bearing;
                hasValidTag = true;
            }
        }
    }

    public int getLastTagId() {
        return lastTagId;
    }

    public double getLastBearing() {
        return lastBearing;
    }

    public boolean hasValidTag() {
        return hasValidTag;
    }


    public void enableProcesor(){
        visionPortal.setProcessorEnabled(aprilTag, true);
    }
    public void disableProcesor(){
        visionPortal.setProcessorEnabled(aprilTag, false);
    }

      /* ================= DATA ================= */

    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    public AprilTagDetection getBestDetection() {
        List<AprilTagDetection> detections = getDetections();
        if (detections.isEmpty()) return null;
        return detections.get(0);
    }


}
