/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "PushServo", group = "TeleOp")
@Config
public class PushServo extends LinearOpMode {
    private FtcDashboard dashboard;

    private Servo servo;
    public static double servoRetractedPos = 0.9;
    public static double servoPushedPos = 0.65;
    public static int waitTimeMs = 500;

    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.get(Servo.class, "pusher");  // numele din configuration
        dashboard = FtcDashboard.getInstance();

        // Set initial position
        servo.setPosition(servoRetractedPos);

        waitForStart();

        while (opModeIsActive()) {

            // Check if button 'a' on gamepad 1 is pressed
            if (gamepad1.a) {
                // Go to pushed position
                servo.setPosition(servoPushedPos);
                // Wait for the specified time
                sleep(waitTimeMs);
                // Return to initial position
                servo.setPosition(servoRetractedPos);
            }

            // You can still adjust positions from the dashboard in real-time.
            // The new values will be used the next time you press the button.

            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.addData("Retracted Pos (Config)", servoRetractedPos);
            telemetry.addData("Pushed Pos (Config)", servoPushedPos);
            telemetry.addData("Wait Time (ms)", waitTimeMs);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Servo Position", servo.getPosition());
            packet.put("Retracted Pos", servoRetractedPos);
            packet.put("Pushed Pos", servoPushedPos);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
