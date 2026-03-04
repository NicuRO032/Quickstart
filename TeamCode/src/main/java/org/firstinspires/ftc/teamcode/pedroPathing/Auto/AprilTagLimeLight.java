
package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Autonomous(name = "Camera", group = "Pedro Pathing")
public class AprilTagLimeLight extends OpMode {

    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); // tag 20
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("BotPose", botPose.toString());
            telemetry.addData("Yaw", botPose.getOrientation().getYaw());
        }
    }
}