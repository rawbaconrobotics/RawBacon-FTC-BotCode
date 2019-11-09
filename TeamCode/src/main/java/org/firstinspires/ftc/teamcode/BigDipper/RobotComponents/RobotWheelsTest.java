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

package org.firstinspires.ftc.teamcode.BigDipper.RobotComponents;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.BigDipper.Robot;
import org.firstinspires.ftc.teamcode.BigDipper.SomeAutonomous;

import static org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.RobotWheels.FRONTLEFT_WHEEL_NAME;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Tank Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

public class RobotWheelsTest extends RobotComponentImplBase {
    final double WHEEL_ACCEL_SPEED_PER_SECOND_STRAIGHT = 2;
    final double WHEEL_DECEL_SPEED_PER_SECOND_STRAIGHT = 15;
    final double WHEEL_ACCEL_SPEED_PER_SECOND_TURNING = 15;
    final double WHEEL_DECEL_SPEED_PER_SECOND_TURNING = 15;
    final double WHEEL_MINIMUM_POWER = 0.3; //Allows for deadband compensation.
    final double WHEEL_MAXIMUM_POWER = 1.0;
    public static boolean DONT_RESET_RUNTIME = false;

    private static final double   COUNTS_PER_MOTOR_REV    = 1440;
    private static final double   DRIVE_GEAR_REDUCTION    = 1.0;
    private static final double   WHEEL_DIAMETER_INCHES   = 4.0;
    private static final double   COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    //Get diameter of turning wheels
    private static final double   OMNIWHEEL_DIAMETER_INCHES  = 4.0;
    //Find circumference of turning wheels
    private static final double   OMNIWHEEL_CIRCUMFERENCE    = OMNIWHEEL_DIAMETER_INCHES * 3.1415;
    //Get distance from center of turning to turning wheels
    private static final double TURNER_TO_DRIVER_INCHES = 9.5;
    //Find the total distance a full spin of the robot covers
    private static final double   TURNER_FLOOR_CIRCUMFERENCE = TURNER_TO_DRIVER_INCHES * 2 * 3.1415;
    //Get drive gear reduction of turning wheels
    private static final double   TURN_DRIVE_GEAR_REDUCTION  = 1.0;
    //Find the number of counts in one turn of the turning wheels
    private static final double   COUNTS_PER_TURNER_TURN     = COUNTS_PER_MOTOR_REV * TURN_DRIVE_GEAR_REDUCTION;
    //Find the number of counts in a full spin of the robot
    private static final double   COUNTS_PER_FULL_TURN = (TURNER_FLOOR_CIRCUMFERENCE / OMNIWHEEL_CIRCUMFERENCE) * COUNTS_PER_TURNER_TURN;
    //Find the number of counts in a degree of a full spin of the robot
    private static final double   COUNTS_PER_DEGREE          = COUNTS_PER_FULL_TURN / 360;



    public DcMotorAccelerationThread wheelAccelerationThread = new DcMotorAccelerationThread();


    DcMotorAccelerated accLeftDriveFront;
    DcMotorAccelerated accLeftDriveBack;
    DcMotorAccelerated accRightDriveFront;
    DcMotorAccelerated accRightDriveBack;



    private final static String FRONTRIGHT_WHEEL_NAME = "right_drive_front";
    private final static String FRONTLEFT_WHEEL_NAME = "left_drive_front";
    private final static String BACKRIGHT_WHEEL_NAME = "right_drive_back";
    private final static String BACKLEFT_WHEEL_NAME = "left_drive_back";


    private static final double SLOW_DRIVE_SCALAR = 0.2;
    private static final double STICK_DIGITAL_THRESHOLD = 0.25;
    private static final double TURNING_SCALAR = 0.875;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDriveBack = null;
    private DcMotor rightDriveBack = null;
    private DcMotor leftDriveFront = null;
    private DcMotor rightDriveFront = null;
    boolean speedModeOn;

    BNO055IMU imu;
    Orientation   lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;



    public RobotWheelsTest(LinearOpMode opMode) {
        super(opMode);
    }


    @Override
    public void init() {
        accLeftDriveFront = new DcMotorAccelerated(opMode.hardwareMap.dcMotor.get(FRONTLEFT_WHEEL_NAME), WHEEL_ACCEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_DECEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_MINIMUM_POWER, WHEEL_MAXIMUM_POWER);
        accLeftDriveBack = new DcMotorAccelerated(opMode.hardwareMap.dcMotor.get(BACKLEFT_WHEEL_NAME), WHEEL_ACCEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_DECEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_MINIMUM_POWER, WHEEL_MAXIMUM_POWER);
        accRightDriveFront = new DcMotorAccelerated(opMode.hardwareMap.dcMotor.get(FRONTRIGHT_WHEEL_NAME), WHEEL_ACCEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_DECEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_MINIMUM_POWER, WHEEL_MAXIMUM_POWER);
        accRightDriveBack = new DcMotorAccelerated(opMode.hardwareMap.dcMotor.get(BACKRIGHT_WHEEL_NAME), WHEEL_ACCEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_DECEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_MINIMUM_POWER, WHEEL_MAXIMUM_POWER);


        leftDriveBack = hardwareMap.dcMotor.get(BACKLEFT_WHEEL_NAME);
        rightDriveBack = hardwareMap.dcMotor.get(BACKRIGHT_WHEEL_NAME);
        leftDriveFront = hardwareMap.dcMotor.get(FRONTLEFT_WHEEL_NAME);
        rightDriveFront = hardwareMap.dcMotor.get(FRONTRIGHT_WHEEL_NAME);

        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.FORWARD);
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);

        leftDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wheelAccelerationThread.addMotor(accLeftDriveFront);
        wheelAccelerationThread.addMotor(accLeftDriveBack);
        wheelAccelerationThread.addMotor(accRightDriveFront);
        wheelAccelerationThread.addMotor(accRightDriveBack);
        wheelAccelerationThread.start();
    }
    @Override
    public void initAutonomous(){
        System.out.println("STARTING TO INIT AUTONOMOUS...");

        leftDriveBack = hardwareMap.dcMotor.get(BACKLEFT_WHEEL_NAME);
        rightDriveBack = hardwareMap.dcMotor.get(BACKRIGHT_WHEEL_NAME);
        leftDriveFront = hardwareMap.dcMotor.get(FRONTLEFT_WHEEL_NAME);
        rightDriveFront = hardwareMap.dcMotor.get(FRONTRIGHT_WHEEL_NAME);

        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.FORWARD);
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);

        leftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        System.out.println("AUTONOMOUS INITIALIZED!");


    }
    /*
     */

    public void wheelsTeleOp() {
        speedModeOn = isBumperPressed();

        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Tank Controller app on the phone).

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery


        // Wait for the game to start (driver presses PLAY)
        //waitForStart();
        //runtime.reset();

        // run until the end of the match (driver presses STOP)
        //while (opModeIsActive()) {

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels

        /*leftDriveBack.setPower(leftPower);
            rightDriveBack.setPower(rightPower);
        leftDriveFront.setPower(leftPower);
        rightDriveFront.setPower(rightPower);
*/



        mechanumTeleOp(gamepad1.left_stick_x,gamepad1.left_stick_y,-gamepad1.right_stick_x);        // Initialize the hardware variables. Note that the strings used here as parameters
        //while (opModeIsActive()) {






    }
    // Show the elapsed game time and wheel power.

    //}

    public void mechanumTeleOp(double x, double y, double rotation) {
        double wheelSpeeds[] = new double[4];





        wheelSpeeds[0] = x + y + rotation;
        wheelSpeeds[1] = -x + y - rotation;
        wheelSpeeds[2] = -x + y + rotation;
        wheelSpeeds[3] = x + y - rotation;

        normalize(wheelSpeeds);
if(speedModeOn) {

    leftDriveBack.setPower(wheelSpeeds[0]);
    rightDriveBack.setPower(wheelSpeeds[1]);
    leftDriveFront.setPower(wheelSpeeds[2]);
    rightDriveFront.setPower(wheelSpeeds[3]);
}
else{
    accLeftDriveBack.setTargetPower(wheelSpeeds[0]);
    accRightDriveBack.setTargetPower(wheelSpeeds[1]);
    accLeftDriveFront.setTargetPower(wheelSpeeds[2]);
    accRightDriveFront.setTargetPower(wheelSpeeds[3]);
}

    }   //mecanumDrive_Cartesian

    private void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);

        for (int i = 1; i < wheelSpeeds.length; i++) {
            double magnitude = Math.abs(wheelSpeeds[i]);

            if (magnitude > maxMagnitude) {
                maxMagnitude = magnitude;
            }
        }

        if (maxMagnitude > 0.75) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }
        //normalize


    }


    //Autonomous methods below

    //Set speeds of turning or driving motors
    public void drive(double speed){
        System.out.println("DRIVE METHOD CALLED, SETTING TO SPEED" + speed);

        leftDriveBack.setPower(speed);
        rightDriveBack.setPower(speed);
        leftDriveFront.setPower(speed);
        rightDriveFront.setPower(speed);
    }

    public void turn(double speed, boolean clockwise){
        System.out.println("TURN METHOD CALLED, SETTING TO SPEED " + speed + " and clockwise = " + clockwise);

        if(clockwise) {
            leftDriveBack.setPower(speed);
            rightDriveBack.setPower(-speed);
            leftDriveFront.setPower(speed);
            rightDriveFront.setPower(-speed);
        }
        else{
            leftDriveBack.setPower(-speed);
            rightDriveBack.setPower(speed);
            leftDriveFront.setPower(-speed);
            rightDriveFront.setPower(speed);
        }
    }

    //Drive for a specified distance using encoders
    public void driveFor(double distance_inches, double speed, double timeoutS) {
        System.out.println("DRIVEFOR METHOD CALLED");

        runUsingEncoders();

        System.out.println("RUNUSINGENCODERS COMPLETE!");

        if (opModeIsActive()) {


            int targetDistLeft;
            int targetDistRight;
            targetDistLeft = leftDriveFront.getCurrentPosition() + (int) (distance_inches * COUNTS_PER_INCH);
            targetDistRight = rightDriveFront.getCurrentPosition() + (int) (distance_inches * COUNTS_PER_INCH);

            leftDriveFront.setTargetPosition(targetDistLeft);
            rightDriveFront.setTargetPosition(targetDistRight);
            leftDriveBack.setTargetPosition(targetDistLeft);
            rightDriveBack.setTargetPosition(targetDistRight);

            System.out.println("SET TARGET POSITIONS");

            leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            System.out.println("SET MODE RUN TO POSITION");

            runtime.reset();

            drive(speed);

            System.out.println("DRIVING AT THAT SPEED");


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDriveFront.isBusy() && rightDriveFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d");
                telemetry.addData("Path2",  "Running at %7d :%7d");
                String s1 = Boolean.toString(leftDriveFront.isBusy());
                String s2 = Boolean.toString(rightDriveFront.isBusy());
                String s3 = Boolean.toString(leftDriveBack.isBusy());
                String s4 = Boolean.toString(rightDriveBack.isBusy());

                telemetry.addData("FRONT LEFT: " + s1," and FRONT RIGHT " + s2);
                telemetry.addData("BACK LEFT: " + s3," and BACK RIGHT " + s4);
                telemetry.update();

                telemetry.update();
                System.out.println("ROBOT SHOULD BE RUNNING NOW");

            }
            drive(0);
            System.out.println("ROBOT STOPPED");

            runUsingEncoders();
            System.out.println("RUN USING ENCODERS METHOD RAN");

        }

    }

    //Turn for a specified amount of degrees using encoders
    public void turnFor ( int degrees, double speed, double timeoutS) {

        System.out.println("TURNFOR METHOD CALLED");


        int targetDistLeft;
        int targetDistRight;
        boolean turningRight = false;
        if (opModeIsActive()) {
            if (degrees > 0) {
                targetDistRight = rightDriveFront.getCurrentPosition() - (int) (degrees * COUNTS_PER_DEGREE);
                targetDistLeft = leftDriveFront.getCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);

                leftDriveFront.setTargetPosition(targetDistLeft);
                leftDriveBack.setTargetPosition(targetDistLeft);
                rightDriveFront.setTargetPosition(targetDistRight);
                rightDriveBack.setTargetPosition(targetDistRight);

                System.out.println("SET TURNING TARGET POS.");


                leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                System.out.println("SET RUN TO POSITION");

            }
            else {
                targetDistRight = rightDriveFront.getCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
                targetDistLeft = leftDriveFront.getCurrentPosition() - (int) (degrees * COUNTS_PER_DEGREE);

                leftDriveFront.setTargetPosition(targetDistLeft);
                leftDriveBack.setTargetPosition(targetDistLeft);
                rightDriveFront.setTargetPosition(targetDistRight);
                rightDriveBack.setTargetPosition(targetDistRight);

                System.out.println("SET TURNING TARGET POS.");


                leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                System.out.println("SET RUN TO POSITION");


            }

            if(degrees > 0){turningRight = true;}
            else{turningRight = false;}

            System.out.println("ABOUT TO RUN TURN COMMAND");
            runtime.reset();

            turn(speed, turningRight);

            System.out.println("TURN COMMAND COMPLETED");



            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDriveFront.isBusy() && rightDriveFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d");
                telemetry.addData("Path2",  "Running at %7d :%7d");
                System.out.println("TURNING THE ROBOT");

                telemetry.update();
            }
            drive(0);
            System.out.println("SPEED SET TO 0");

            runUsingEncoders();
            System.out.println("RAN RUNUSINGENCODERS");

        }
    }



    public void runUsingEncoders(){
        System.out.println("ABOUT TO SET RUNUSINGENCODERS DIRECTLY...");

        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        System.out.println("SET IT DIRECTLY!");

    }



    public void stopDrive(){
        wheelAccelerationThread.stop();
        System.out.println("STOPDRIVE completed");

    }

//AUTONOMOUS STUFF
    /*
    public double gyroStrafeNormalized(double pow, double target, double Kp)
    {
        double err = getAngle() - target;
        MecanumDrive.cartesian(driveTrain, 0, pow, err*Kp);
        return err;
    }

    public double gyroStraightNormalized(double pow, double target, double Kp)
    {
        double err = getAngle() - target;

        MecanumDrive.cartesian(driveTrain, pow, 0, err*Kp);

        return err;
    }

    public static void cartesian(DriveTrain driveTrain, double mainSpeed, double strafeSpeed, double turnSpeed){

    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
public void normalizeAuto(){
    /*
     * Are any of the computed wheel powers greater than 1?
     */
    /*
    if(Math.abs(FL_power_raw) > 1
            || Math.abs(FR_power_raw) > 1
            || Math.abs(RL_power_raw) > 1
            || Math.abs(RR_power_raw) > 1)
    {
        /*
         * Yeah, figure out which one
         *//*
        maxLeft = Math.max(Math.abs(FL_power_raw), Math.abs(RL_power_raw));
        maxRight = Math.max(Math.abs(FR_power_raw), Math.abs(RR_power_raw));
        max = Math.max(maxLeft, maxRight);

        ratio = 1 / max; //Create a ratio to normalize them all

        motorPowers.frontLeft  = FL_power_raw * ratio;
        motorPowers.frontRight = FR_power_raw * ratio;
        motorPowers.rearLeft   = RL_power_raw * ratio;
        motorPowers.rearRight  = RR_power_raw * ratio;
    }

    /*
     * Nothing we need to do to the raw powers
     *//*
    else
    {
        motorPowers.frontLeft = FL_power_raw;
        motorPowers.frontRight = FR_power_raw;
        motorPowers.rearLeft = RL_power_raw;
        motorPowers.rearRight = RR_power_raw;
    }
}
*/
boolean isBumperPressed(){
    float bumperNumber = gamepad1.right_trigger;
    boolean bumperPressed;
    if(bumperNumber > 0.3){
        bumperPressed = true;
    }
    else{
        bumperPressed = false;
    }
    return bumperPressed;
}

}





