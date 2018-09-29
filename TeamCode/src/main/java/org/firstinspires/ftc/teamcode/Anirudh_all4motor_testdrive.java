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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This OpMode ramps a single motor speed up and down repeatedly until Stop is pressed.
 * The code is structured as a LinearOpMode
 *
 * This code assumes a DC motor configured with the name "left_drive" as is found on a pushbot.
 *
 * INCREMENT sets how much to increase/decrease the power each cycle
 * CYCLE_MS sets the update period.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Anirudh_motor1and2_testdrive.java", group = "Concept")

public class Anirudh_all4motor_testdrive extends LinearOpMode {

    static final double INCREMENT = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_FWD = 1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV = -1.0;     // Maximum REV power applied to motor

    // Define class members
    DcMotor motor1;
    double power1 = 0;
    DcMotor motor2;
    double power2 = 0;
    DcMotor motor3;
    double power3 = 0;
    DcMotor motor4;
    double power4 = 0;
    double max;
    double forward;
    double clockwise;
    double right;

    @Override
    public void runOpMode() {

        // Connect to motor (Assume standard left wheel)
        // Change the text in quotes to match any motor name on your robot.
        motor1 = hardwareMap.get(DcMotor.class, "motor1");

        motor2 = hardwareMap.get(DcMotor.class, "motor2");

        motor3 = hardwareMap.get(DcMotor.class, "motor3");

        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors.");
        telemetry.update();

        // Set Default Motor Directions
        motor1.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motor2.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
        motor3.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motor4.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors

        waitForStart();

        // Ramp motor speeds till stop pressed.
        while (opModeIsActive()) {

            //Drive Via "Analog Sticks" (Not Toggle)
            //Set initial motion parameters to Gamepad1 Inputs
            forward = -gamepad1.left_stick_y;
            //right = gamepad1.left_stick_x;
            right = -gamepad1.left_trigger + gamepad1.right_trigger;
            clockwise = gamepad1.right_stick_x;

            power1 = forward + clockwise + right;
            power2 = forward - clockwise - right;
            power3 = forward + clockwise - right;
            power4 = forward - clockwise + right;

            // Normalize Wheel speeds so that no speed exceeds 1.0
            max = Math.abs(power1);
            if (Math.abs(power2) > max) {
                max = Math.abs(power2);
            }
            if (Math.abs(power3) > max) {
                max = Math.abs(power3);
            }
            if (Math.abs(power4) > max) {
                max = Math.abs(power4);
            }

            if (max > 1) {
                power1 /= max;
                power2 /= max;
                power3 /= max;
                power4 /= max;
            }

            motor1.setPower(power1);
            motor2.setPower(power2);
            motor3.setPower(power3);
            motor4.setPower(power4);


                // Display the current value
                telemetry.addData("Motor Power", "%5.2f", power1);
                telemetry.addData(">", "Press Stop to end test.");
                telemetry.update();

                telemetry.addData("Motor Power", "%5.2f", power2);
                telemetry.addData(">", "Press Stop to end test.");
                telemetry.update();

                telemetry.addData("Motor Power", "%5.2f", power3);
                telemetry.addData(">", "Press Stop to end test.");
                telemetry.update();

                telemetry.addData("Motor Power", "%5.2f", power4);
                telemetry.addData(">", "Press Stop to end test.");
                telemetry.update();


                idle();
            }

            // Turn off motor and signal done;
            motor1.setPower(0);
            motor2.setPower(0);
            motor3.setPower(0);
            motor4.setPower(0);
            telemetry.addData(">", "Done");
            telemetry.update();

        }
    }


