
package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Camera", group = "Pedro Pathing")
public class AprilTagLimeLight extends OpMode {

    private Limelight3A limelight;

    TestBench bench = new TestBench();

    private double distance;

    @Override
    public void init() {
        bench.init(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); // tag 20
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = bench.getOrientation();
        limelight.updateRobotOrientation(orientation != null ? orientation.getYaw(AngleUnit.DEGREES) : 0);
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            distance = getDistanceFromTag(llResult.getTa());
            telemetry.addData("Distance", distance);
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("BotPose", botPose.toString());
        }
    }
     public double getDistanceFromTag(double ta) {
        double scale = 32779.75 / 100;
        double distance = (scale / ta);
        return distance;
     }

    private class TestBench {
        public void init(HardwareMap hardwareMap) {
        }

        public YawPitchRollAngles getOrientation() {

        return null;
        }
    }
}