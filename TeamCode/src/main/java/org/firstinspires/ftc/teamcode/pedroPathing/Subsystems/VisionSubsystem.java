package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collections;
import java.util.List;

public class VisionSubsystem extends SubsystemBase {

    /* ================= CAMERA POSE ================= */

    private final Position cameraPosition = new Position(
            DistanceUnit.INCH, 0, 0, 0, 0);

    private final YawPitchRollAngles cameraOrientation =
            new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    /* ================= VISION ================= */

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private boolean enabled = false;

    /* ================= CONSTRUCTOR ================= */

    public VisionSubsystem() {
        // ⚠️ NU atinge camera aici
    }

    /* ================= ENABLE / DISABLE ================= */

    public void enable(HardwareMap hardwareMap) {
        if (enabled) return;

        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();

        enabled = true;
    }

    public void disable() {
        if (!enabled) return;

        visionPortal.close();   // 🔴 foarte important
        visionPortal = null;
        aprilTag = null;
        enabled = false;
    }

    public boolean isEnabled() {
        return enabled;
    }

    /* ================= DATA ================= */

    public List<AprilTagDetection> getDetections() {
        if (!enabled || aprilTag == null) return Collections.emptyList();
        return aprilTag.getDetections();
    }

    public AprilTagDetection getBestDetection() {
        List<AprilTagDetection> detections = getDetections();
        if (detections.isEmpty()) return null;
        return detections.get(0);
    }
}
