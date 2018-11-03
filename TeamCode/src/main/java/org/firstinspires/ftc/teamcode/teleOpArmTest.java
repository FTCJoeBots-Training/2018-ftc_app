package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 *import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 *import com.qualcomm.robotcore.hardware.DcMotor;
 *
 *
 */

/**
 *Notes For this TeleOp Code. This code is for Comp and all proggramers should review over this
 *code and understand this code for the possibility that a question may be asked related to TeleOp and
 *you should be able to explain in good detail everything in this code.
 *11/16/17-> Changed all gamepad's in code to correct gamepad (i.e some gamepad1's to gamepad2)
 ***11/18/17-> Competition Notes below
 *Notes-> Autonomous is incorrect, Not much was wrong from a software sandpoint but hardware issues were fixed
 *Autonomous issues included: Incorrect spinning causing us to move out of destination,
 *To much time on the down motion of the clamp and arm.
 *These issues are still not resolved
 * Recomendation for autonomous issues(Not Offical):Fine tune the timer on the clamp
 * Fine tune the movements and LOWER the TIME OF MOVEMENT in autonomous.
 * List of issues at Comp(1)-> https://docs.google.com/a/stjoebears.com/spreadsheets/d/1r_liipKBU7GHfONdxq9E6d4f7zikcCuXwDL2bsQfwm0/edit?usp=sharing
 *G-Sheet of time VS Heading for autonomous -> https://docs.google.com/a/stjoebears.com/spreadsheets/d/1pqv0iN94fFd5KvX1YIWP7z39HgpURXsscn0zPujs1q4/edit?usp=sharing
*/
@TeleOp(name="Simple Arm Test", group="TeleOp")

public class teleOpArmTest extends LinearOpMode {

    HardwareMap hwMap = null;
    DcMotor armMotor = null; // Arm Motor


    @Override
    public void runOpMode() throws InterruptedException {

        hwMap = hardwareMap;

        armMotor = hwMap.dcMotor.get("armMotor");

        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setPower(0);

        double armPower = 0;

        waitForStart();


        //start of loop
        while (opModeIsActive()) {


            armPower = gamepad1.right_stick_y;

            armMotor.setPower(armPower);


            // Update Telemetry
            telemetry.addLine("Use Right-Stick-Y to drive Arm");
            telemetry.addData("Arm Power: ", armPower);
            telemetry.update();
            idle();


        }//end while
    }
}