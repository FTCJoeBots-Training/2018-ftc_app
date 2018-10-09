package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 *import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 *import com.qualcomm.robotcore.hardware.DcMotor;
 *
 *
 */

@TeleOp(name="Mecanum Move Method Test", group="TeleOp")

public class teleOpSimpleMecanumTest extends LinearOpMode {

    HardwareJoeBotTest robot = new HardwareJoeBotTest();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, this);


        double forward;
        double clockwise;
        double right;


        waitForStart();



        //start of loop
        while (opModeIsActive()) {


            robot.moveRobot(-gamepad1.left_stick_y, (-gamepad1.left_trigger+gamepad1.right_trigger), gamepad1.right_stick_x);



            // Update Telemetry
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();
            idle();






        }//end while
    }
}