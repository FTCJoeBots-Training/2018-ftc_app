package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode. This is a hardware class used to abstract the hardware config for the
 * 2018 JoeBots FTC Rover Ruckus challenge. This file has been generalized to work as a base for
 * all three JoeBots FTC teams (8513, 11855, and 13702). As the season progresses, this file may be
 * customized for each individual team in their own branch.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 *
 * motor1 (left front)
 * motor2 (right front)
 * motor3 (left rear)
 * motor4 (right rear)
 * imu - navigation features
 *
 * Note:  All names are lower case and some have single spaces between words.
 *
 */

public class HardwareJoeBotTest
{
    /* Public OpMode members. */

    // Declare Motors
    public DcMotor  motor1 = null; // Left Front
    public DcMotor  motor2 = null; // Right Front
    public DcMotor  motor3 = null; // Left Rear
    public DcMotor  motor4 = null; // Right Rear

    // Declare Sensors
    public BNO055IMU imu;                  // The IMU sensor object

    // Variables used for IMU tracking...
    public Orientation angles;
    public Acceleration gravity;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    // Private Members
    private LinearOpMode myOpMode;

    // Declare static members for calculations
    static final double COUNTS_PER_MOTOR_REV    = 1120;
    static final double DRIVE_GEAR_REDUCTION    = 1;
    static final double WHEEL_DIAMETER_INCHES   = 4;
    static final double COUNTS_PER_INCH         =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/
                    (WHEEL_DIAMETER_INCHES * 3.14159);


    /* Constructor */
    public HardwareJoeBotTest(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode opMode) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        myOpMode = opMode;

        // Define and Initialize Motors
        motor1 = hwMap.dcMotor.get("motor1");
        motor2 = hwMap.dcMotor.get("motor2");
        motor3 = hwMap.dcMotor.get("motor3");
        motor4 = hwMap.dcMotor.get("motor4");

        // Set Default Motor Directions
        motor1.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motor2.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
        motor3.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motor4.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);

        // Set all drive motors to run without encoders.
        // May want to switch to  RUN_USING_ENCODERS during autonomous
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // IMU Initializaiton
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }


    /***
     * void setMode(DcMotor.RunMode mode ) Set all drive motors to same mode.
     * @param mode    Desired Motor mode.
     */
    public void setMode(DcMotor.RunMode mode ) {
        motor1.setMode(mode);
        motor2.setMode(mode);
        motor3.setMode(mode);
        motor4.setMode(mode);
    }

    public void moveRobot(double forward, double right, double clockwise) {

        // Code should take input from method and calculate power for each of the mecanum drive
        // motors. Code should then set the power to the motors.

        //declare power variables
        double power1;
        double power2;
        double power3;
        double power4;
        double max;

        // Set Power Variables equal to mecanum formulas
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

        // Set power to motors
        motor1.setPower(power1);
        motor2.setPower(power2);
        motor3.setPower(power3);
        motor4.setPower(power4);



    }

    public void stop() {
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);

    }


    public void moveInches(double inches, double power, int timeoutSec){

        int newMotor1Target;
        int newMotor2Target;
        int newMotor3Target;
        int newMotor4Target;

        if(myOpMode.opModeIsActive()){

            newMotor1Target = motor1.getCurrentPosition() + (int)(inches*COUNTS_PER_INCH);
            newMotor2Target = motor2.getCurrentPosition() + (int)(inches*COUNTS_PER_INCH);
            newMotor3Target = motor3.getCurrentPosition() + (int)(inches*COUNTS_PER_INCH);
            newMotor4Target = motor4.getCurrentPosition() + (int)(inches*COUNTS_PER_INCH);

            motor1.setTargetPosition(newMotor1Target);
            motor2.setTargetPosition(newMotor2Target);
            motor3.setTargetPosition(newMotor3Target);
            motor4.setTargetPosition(newMotor4Target);

            setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            moveRobot(power,0,0);

            while (myOpMode.opModeIsActive() && runtime.seconds() < timeoutSec &&
                    (motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && motor4.isBusy())){
            }

            stop();

            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }


    }

}






