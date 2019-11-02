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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BigDipper.Robot;

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
    final double WHEEL_ACCEL_SPEED_PER_SECOND_STRAIGHT = 0.8;
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
    DcMotorAccelerated accLeftDriveFront = new DcMotorAccelerated(opMode.hardwareMap.dcMotor.get(FRONTLEFT_WHEEL_NAME), WHEEL_ACCEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_DECEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_MINIMUM_POWER, WHEEL_MAXIMUM_POWER);
    DcMotorAccelerated accLeftDriveBack = new DcMotorAccelerated(opMode.hardwareMap.dcMotor.get(BACKLEFT_WHEEL_NAME), WHEEL_ACCEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_DECEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_MINIMUM_POWER, WHEEL_MAXIMUM_POWER);
    DcMotorAccelerated accRightDriveFront = new DcMotorAccelerated(opMode.hardwareMap.dcMotor.get(FRONTRIGHT_WHEEL_NAME), WHEEL_ACCEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_DECEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_MINIMUM_POWER, WHEEL_MAXIMUM_POWER);
    DcMotorAccelerated accRightDriveBack = new DcMotorAccelerated(opMode.hardwareMap.dcMotor.get(BACKRIGHT_WHEEL_NAME), WHEEL_ACCEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_DECEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_MINIMUM_POWER, WHEEL_MAXIMUM_POWER);



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
    private DcMotor rightDriveFront = null;
    private DcMotor leftDriveFront = null;


    public RobotWheelsTest(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init() {

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


    }

    public void wheelsTeleOp() {

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

        accLeftDriveBack.setTargetPower(wheelSpeeds[0]);
        accRightDriveBack.setTargetPower(wheelSpeeds[1]);
        accLeftDriveFront.setTargetPower(wheelSpeeds[2]);
        accRightDriveFront.setTargetPower(wheelSpeeds[3]);

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

        leftDriveBack.setPower(speed);
        rightDriveBack.setPower(speed);
        leftDriveFront.setPower(speed);
        rightDriveFront.setPower(speed);
    }

    public void turn(double speed, boolean clockwise){
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
    public void driveFor(double distance_inches, double speed) {

        runUsingEncoders();
        if (opModeIsActive()) {


            int targetDistLeft;
            int targetDistRight;
            targetDistLeft = leftDriveFront.getCurrentPosition() + (int) (distance_inches * COUNTS_PER_INCH);
            targetDistRight = rightDriveFront.getCurrentPosition() + (int) (distance_inches * COUNTS_PER_INCH);

            leftDriveFront.setTargetPosition(targetDistLeft);
            rightDriveFront.setTargetPosition(targetDistRight);
            leftDriveBack.setTargetPosition(targetDistLeft);
            rightDriveBack.setTargetPosition(targetDistRight);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive(speed);

            while (opModeIsActive() &&
                    (runtime.seconds() < 15) &&
                    (leftDriveFront.isBusy() && rightDriveFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d");
                telemetry.addData("Path2",  "Running at %7d :%7d");

                telemetry.update();
            }
            drive(0);
            runUsingEncoders();
        }

    }

    //Turn for a specified amount of degrees using encoders
    public void turnFor ( int degrees, double speed) {

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

                leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else {
                targetDistRight = rightDriveFront.getCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
                targetDistLeft = leftDriveFront.getCurrentPosition() - (int) (degrees * COUNTS_PER_DEGREE);

                leftDriveFront.setTargetPosition(targetDistLeft);
                leftDriveBack.setTargetPosition(targetDistLeft);
                rightDriveFront.setTargetPosition(targetDistRight);
                rightDriveBack.setTargetPosition(targetDistRight);

                leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            if(degrees > 0){turningRight = true;}
            else{turningRight = false;}

            turn(speed, turningRight);

            while (opModeIsActive() &&
                    (runtime.seconds() < 15) &&
                    (leftDriveFront.isBusy() && rightDriveFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d");
                telemetry.addData("Path2",  "Running at %7d :%7d");

                telemetry.update();
            }
            drive(0);
            runUsingEncoders();
        }
    }



    public void runUsingEncoders(){

        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    public void stopDrive(){
        wheelAccelerationThread.stop();
    }

}



