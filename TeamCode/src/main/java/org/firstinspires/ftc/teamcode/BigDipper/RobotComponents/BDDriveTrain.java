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

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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

import java.time.temporal.ValueRange;
import java.util.stream.IntStream;

import static android.os.SystemClock.sleep;

public class BDDriveTrain extends RobotComponentImplBase {
    final double WHEEL_ACCEL_SPEED_PER_SECOND_STRAIGHT = 2;
    final double WHEEL_DECEL_SPEED_PER_SECOND_STRAIGHT = 15;
    final double WHEEL_MINIMUM_POWER = 0.3; //Allows for deadband compensation.
    final double WHEEL_MAXIMUM_POWER = 1.0;

    private static final double   COUNTS_PER_MOTOR_REV    = 1120; //1440
    private static final double   DRIVE_GEAR_REDUCTION    = 1.0;
    private static final double   WHEEL_DIAMETER_INCHES   = 4.0;
    private static final double   COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * 3.1415);

    private static final double   COUNTS_PER_DEGREE          = 15;

    public double currentSpeed;


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

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDriveBack = null;
    private DcMotor rightDriveBack = null;
    private DcMotor leftDriveFront = null;
    private DcMotor rightDriveFront = null;
    boolean speedModeOn;

    BNO055IMU imu;
    Orientation   lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;

    double realAngle = 0;
    double degreesWanted = 0;




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

        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);

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


        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);

        leftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        currentSpeed = Math.abs(leftDriveBack.getPower());

        //leftDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
 imu = hardwareMap.get(BNO055IMU.class, "imu");
 imu.initialize(parameters);

        //  wheelAccelerationThread.addMotor(accLeftDriveFront);
        // wheelAccelerationThread.addMotor(accLeftDriveBack);
        // wheelAccelerationThread.addMotor(accRightDriveFront);
        // wheelAccelerationThread.addMotor(accRightDriveBack);
        //wheelAccelerationThread.start();

        runUsingEncoders();

    }

    public void wheelsTeleOp() {
        speedModeOn = isBumperPressed();

        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        mechanumTeleOp(gamepad1.left_stick_x,gamepad1.left_stick_y,-gamepad1.right_stick_x);
    }

    public void mechanumTeleOp(double x, double y, double rotation) {
        double wheelSpeeds[] = new double[4];

        wheelSpeeds[0] = x + y + rotation;
        wheelSpeeds[1] = -x + y - rotation;
        wheelSpeeds[2] = -x + y + rotation;
        wheelSpeeds[3] = x + y - rotation;

        normalize(wheelSpeeds);
        if(speedModeOn) {
            leftDriveBack.setPower(Range.clip((wheelSpeeds[0]), -1, 1));
            rightDriveBack.setPower(Range.clip((wheelSpeeds[1]), -1, 1));
            leftDriveFront.setPower(Range.clip((wheelSpeeds[2]), -1, 1));
            rightDriveFront.setPower(Range.clip((wheelSpeeds[3]), -1, 1));
        }
        else{
            accLeftDriveBack.setTargetPower(wheelSpeeds[0]);
            accRightDriveBack.setTargetPower(wheelSpeeds[1]);
            accLeftDriveFront.setTargetPower(wheelSpeeds[2]);
            accRightDriveFront.setTargetPower(wheelSpeeds[3]);
        }
    }

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

    public void strafe(double speed, boolean strafingLeft){
        if(strafingLeft) {
            leftDriveBack.setPower(speed);
            rightDriveBack.setPower(-speed);
            leftDriveFront.setPower(-speed);
            rightDriveFront.setPower(speed);
        }

        else{

            leftDriveBack.setPower(-speed);
            rightDriveBack.setPower(speed);
            leftDriveFront.setPower(speed);
            rightDriveFront.setPower(-speed);
/*
            accLeftDriveBack.setTargetPower(-speed);
            accRightDriveBack.setTargetPower(speed);
            accLeftDriveFront.setTargetPower(speed);
            accRightDriveFront.setTargetPower(-speed);
            */

        }
    }

    public void turn(double speed, boolean clockwise){
        System.out.println("TURN METHOD CALLED, SETTING TO SPEED " + speed + " and clockwise = " + clockwise);

        if(clockwise) {

            leftDriveBack.setPower(speed);
            rightDriveBack.setPower(-speed);
            leftDriveFront.setPower(speed);
            rightDriveFront.setPower(-speed);
            /*
            accLeftDriveBack.setTargetPower(speed);
            accRightDriveBack.setTargetPower(-speed);
            accLeftDriveFront.setTargetPower(speed);
            accRightDriveFront.setTargetPower(-speed);
            */

        }
        else{

            leftDriveBack.setPower(-speed);
            rightDriveBack.setPower(speed);
            leftDriveFront.setPower(-speed);
            rightDriveFront.setPower(speed);
/*
            accLeftDriveBack.setTargetPower(-speed);
            accRightDriveBack.setTargetPower(speed);
            accLeftDriveFront.setTargetPower(-speed);
            accRightDriveFront.setTargetPower(speed);
           //hello ignore this message
            */

        }
    }
    public double betterDrive(double speed){
        currentSpeed = Math.abs(leftDriveBack.getPower());
        double MaxAccel = 0.1;
        double deltaTime = 0.1;
        double rawChange = (MaxAccel * deltaTime);
        double targetSpeed = speed;
        if(targetSpeed > currentSpeed){
            currentSpeed = Math.min(targetSpeed, currentSpeed + rawChange);
        }
        else{
            currentSpeed = Math.max(targetSpeed, currentSpeed - rawChange);
        }

        return currentSpeed;

    }

    //Drive for a specified distance using encoders
    public void driveFor(double distance_inches, double speed, double timeoutS) {

        System.out.println("DRIVEFOR METHOD CALLED");

        //runUsingEncoders();

        //System.out.println("RUNUSINGENCODERS COMPLETE!");

        if (opModeIsActive()) {
            double speedWeWant = betterDrive(speed);

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

            drive(speedWeWant);
            //drive(speed);
            System.out.println("DRIVING AT THAT SPEED");

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDriveFront.isBusy() || rightDriveFront.isBusy() || rightDriveBack.isBusy() || leftDriveBack.isBusy()))
            {


                if(speedWeWant != leftDriveBack.getPower()){
                    speedWeWant = betterDrive(speed);
                    drive(speedWeWant);
                    sleep(100);
                }
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
    public void strafeFor(double distance_inches, double speed, boolean strafingLeft, double timeoutS) {
        System.out.println("DRIVEFOR METHOD CALLED");

        //runUsingEncoders();

        //System.out.println("RUNUSINGENCODERS COMPLETE!");

        if (opModeIsActive()) {

            int targetDist;


            leftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            targetDist = leftDriveFront.getCurrentPosition() + (int) (distance_inches * COUNTS_PER_INCH);
            double currentSpeed = betterDrive(speed);
            if(strafingLeft){
                leftDriveBack.setTargetPosition(targetDist);
                rightDriveBack.setTargetPosition(-targetDist);
                rightDriveFront.setTargetPosition(targetDist);
                leftDriveFront.setTargetPosition(-targetDist);
            }
            else{
                leftDriveBack.setTargetPosition(-targetDist);
                rightDriveBack.setTargetPosition(targetDist);
                rightDriveFront.setTargetPosition(-targetDist);
                leftDriveFront.setTargetPosition(targetDist);
            }

            System.out.println("SET TARGET POSITIONS");

            leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            System.out.println("SET MODE RUN TO POSITION");

            runtime.reset();

            strafe(currentSpeed, strafingLeft);

            System.out.println("DRIVING AT THAT SPEED");


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDriveBack.isBusy() && rightDriveBack.isBusy())) {

                if(currentSpeed != leftDriveBack.getPower()){
                    currentSpeed = betterDrive(speed);
                    drive(currentSpeed);
                    sleep(100);
                }


                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d");
                telemetry.addData("Path2",  "Running at %7d :%7d");
                String s3 = Boolean.toString(leftDriveBack.isBusy());
                String s4 = Boolean.toString(rightDriveBack.isBusy());

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
        leftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    //Turn for a specified amount of degrees using encoders
    public void turnFor ( int degrees, double speed, double timeoutS) {
        System.out.println("TURNFOR METHOD CALLED");
    degreesWanted = degrees;

        int targetDistLeft;
        int targetDistRight;
        boolean turningRight = false;
        if (opModeIsActive()) {
            if (degrees > 0) {
                targetDistRight = rightDriveFront.getCurrentPosition() - (int) (degrees * COUNTS_PER_DEGREE * COUNTS_PER_MOTOR_REV);
                targetDistLeft = leftDriveFront.getCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE * COUNTS_PER_MOTOR_REV);

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

           // headingAngle = getAngle();

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
            realAngle = getAngle();

            double acceptableAngleError = 10;

            if (realAngle < (degreesWanted - acceptableAngleError) || realAngle > (degreesWanted + acceptableAngleError)){
System.out.println("within range, no change needed");
            }
            else{
                turnFor((int)(degreesWanted - realAngle), 0.2, 5);
            }

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
    /**
public void normalizeAuto(){
    /*
     * Are any of the computed wheel powers greater than 1?
     */

    /**
    if(Math.abs(FL_power_raw) > 1
            || Math.abs(FR_power_raw) > 1
            || Math.abs(RL_power_raw) > 1
            || Math.abs(RR_power_raw) > 1)
    {

         // Yeah, figure out which one
         //*
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
     */
    /**
    else
    {
        motorPowers.frontLeft = FL_power_raw;
        motorPowers.frontRight = FR_power_raw;
        motorPowers.rearLeft = RL_power_raw;
        motorPowers.rearRight = RR_power_raw;
    }
}
**/
    boolean isBumperPressed(){
        float bumperNumber = gamepad1.right_trigger;
        boolean bumperPressed;
        bumperPressed = bumperNumber > 0.3;
        return bumperPressed;
    }

    public BDDriveTrain(LinearOpMode opMode) {
        super(opMode);
    }

}


