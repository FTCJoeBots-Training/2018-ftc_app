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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
@TeleOp(name = "Mariah Ramp Motor Speed", group = "Concept")
//@Disabled
public class MariahMotorMaping extends LinearOpMode {

    static final double INCREMENT = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_FWD = 1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV = -1.0;     // Maximum REV power applied to motor

    // Define class members
    DcMotor leftmotor;
    DcMotor leftmotor2;
    double leftpower = 0;
    DcMotor rightmotor;
    DcMotor rightmotor2;
    double rightpower = 0;


    @Override
    public void runOpMode() {

        // Connect to motor (Assume standard left wheel)
        // Change the text in quotes to match any motor name on your robot.
        //All Moter move in teleop
        leftmotor = hardwareMap.get(DcMotor.class, "motor1");
        leftmotor2 = hardwareMap. get(DcMotor.class, "motor3");
        rightmotor = hardwareMap.get(DcMotor.class, "motor2");
        rightmotor2 = hardwareMap. get(DcMotor.class, "motor4");


        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors.");
        telemetry.update();
        waitForStart();

        // Ramp motor speeds till stop pressed.
        while (opModeIsActive()) {

            // Read data from gamepad
            leftpower = gamepad1.left_stick_y;
            rightpower=-gamepad1.right_stick_y;
            // Display the current value
            telemetry.addData("leftMotor Power", "%5.2f", leftpower);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            // Set the motor to the new power and pause;
            leftmotor.setPower(leftpower);
            leftmotor2.setPower(leftpower);
            rightmotor.setPower(rightpower);
            rightmotor2.setPower(rightpower);
            sleep(CYCLE_MS);
            idle();
        }

        // Turn off motor and signal done;
        leftmotor.setPower(0);
        leftmotor2.setPower(0);
        rightmotor.setPower(0);
        rightmotor2.setPower(0);
        telemetry.addData(">", "Done");
        telemetry.update();

    }

}

