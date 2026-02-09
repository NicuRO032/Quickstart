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


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard; // ADAUGAT
import com.acmerobotics.dashboard.telemetry.TelemetryPacket; // ADAUGAT

@TeleOp(name = "ConceptScanServo", group = "TeleOp")
@Config
public class ConceptScanServo extends LinearOpMode {

    private Servo servo;
    private AnalogInput analogFeedback;
    private FtcDashboard dashboard;
    private double feedbackVoltage;
    public static double servoPos = 0.0;   // poziția inițială (poți schimba)

    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.get(Servo.class, "carouselServo");  // numele din configuration
        analogFeedback = hardwareMap.get(AnalogInput.class, "axonFeedback"); // numele intrării analogice
        dashboard = FtcDashboard.getInstance();
        //servo.setPosition(servoPos);
        servoPos = servo.getPosition();
        telemetry.addData("Incep de la pozitia", servo.getPosition());
        telemetry.addData("Cu Analog Feedback (4th Wire)", "%.3f V", feedbackVoltage);
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {

            // Creștere poziție: buton Y
            if (gamepad1.y) {
                servoPos += 0.01;
                sleep(100);  // ca să nu adauge prea repede
            }

            // Scădere poziție: buton X
            if (gamepad1.x) {
                servoPos -= 0.01;
                sleep(100);
            }
            if (gamepad1.a) {
                servo.setPosition(0.5);
            }

            // Limitare poziție între 0 și 1
            servoPos = Math.max(0, Math.min(1, servoPos));

            // Aplică poziția
            servo.setPosition(servoPos);
            feedbackVoltage = analogFeedback.getVoltage();

            telemetry.addData("Servo Position", servoPos);
            telemetry.addData("2. Analog Feedback (4th Wire)", "%.3f V", feedbackVoltage);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Commanded Position", servoPos);
            packet.put("Analog Feedback Voltage", feedbackVoltage);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
