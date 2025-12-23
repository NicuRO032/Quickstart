package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
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
                .build();
    }

    @Override
    public void periodic() {
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
