
package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class AprilTagLimeLight extends OpMode {

    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(9);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

    }
}