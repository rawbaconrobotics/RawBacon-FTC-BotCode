package org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.time.temporal.ValueRange;
import java.util.stream.IntStream;

import static android.os.SystemClock.sleep;


/**
 * Represents the four wheel mechanum drive on Uhaul
 * @author Raw Bacon Coders
 */
public class UhaulDriveTrain extends UhaulComponentImplBase {

    double kp = 0.3;
    double ki = 0.1;
    double kd = 0;
    double kf = 12.6;

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
    private DcMotorEx leftDriveBack = null;
    private DcMotorEx rightDriveBack = null;
    private DcMotorEx leftDriveFront = null;
    private DcMotorEx rightDriveFront = null;
    boolean speedModeOn;

    BNO055IMU imu;
    Orientation   lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    double realAngle = 0;
    double degreesWanted = 0;




    /**
     * Hardware maps and sets modes of all motors
     */
    @Override
    public void init() {
        accLeftDriveFront = new DcMotorAccelerated(opMode.hardwareMap.dcMotor.get(FRONTLEFT_WHEEL_NAME), WHEEL_ACCEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_DECEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_MINIMUM_POWER, WHEEL_MAXIMUM_POWER);
        accLeftDriveBack = new DcMotorAccelerated(opMode.hardwareMap.dcMotor.get(BACKLEFT_WHEEL_NAME), WHEEL_ACCEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_DECEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_MINIMUM_POWER, WHEEL_MAXIMUM_POWER);
        accRightDriveFront = new DcMotorAccelerated(opMode.hardwareMap.dcMotor.get(FRONTRIGHT_WHEEL_NAME), WHEEL_ACCEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_DECEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_MINIMUM_POWER, WHEEL_MAXIMUM_POWER);
        accRightDriveBack = new DcMotorAccelerated(opMode.hardwareMap.dcMotor.get(BACKRIGHT_WHEEL_NAME), WHEEL_ACCEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_DECEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_MINIMUM_POWER, WHEEL_MAXIMUM_POWER);


        leftDriveBack = (DcMotorEx) hardwareMap.dcMotor.get(BACKLEFT_WHEEL_NAME);
        rightDriveBack = (DcMotorEx) hardwareMap.dcMotor.get(BACKRIGHT_WHEEL_NAME);
        leftDriveFront = (DcMotorEx) hardwareMap.dcMotor.get(FRONTLEFT_WHEEL_NAME);
        rightDriveFront = (DcMotorEx) hardwareMap.dcMotor.get(FRONTRIGHT_WHEEL_NAME);

        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.FORWARD);
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);

        leftDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDFCoefficients pidNew = new PIDFCoefficients(kp, ki, kd, kf);
        leftDriveFront.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        leftDriveBack.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        rightDriveFront.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        rightDriveBack.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);


        wheelAccelerationThread.addMotor(accLeftDriveFront);
        wheelAccelerationThread.addMotor(accLeftDriveBack);
        wheelAccelerationThread.addMotor(accRightDriveFront);
        wheelAccelerationThread.addMotor(accRightDriveBack);
        wheelAccelerationThread.start();
    }

    /**
     * Hardware maps and sets modes of all motors and sets up the imu
     */
    @Override
    public void initAutonomous(){
        System.out.println("STARTING TO INIT AUTONOMOUS...");

        leftDriveBack = (DcMotorEx) hardwareMap.dcMotor.get(BACKLEFT_WHEEL_NAME);
        rightDriveBack = (DcMotorEx) hardwareMap.dcMotor.get(BACKRIGHT_WHEEL_NAME);
        leftDriveFront = (DcMotorEx) hardwareMap.dcMotor.get(FRONTLEFT_WHEEL_NAME);
        rightDriveFront = (DcMotorEx) hardwareMap.dcMotor.get(FRONTRIGHT_WHEEL_NAME);


        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);

        leftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        PIDFCoefficients pidNew = new PIDFCoefficients(kp, ki, kd, kf);
        leftDriveFront.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        leftDriveBack.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        rightDriveFront.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        rightDriveBack.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

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

    /**
     * Reformats input and runs the {@link #mechanumTeleOp} method
     */
    public void wheelsTeleOp() {
        speedModeOn = isBumperPressed();

        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        mechanumTeleOp(gamepad1.left_stick_x,gamepad1.left_stick_y,-gamepad1.right_stick_x);
    }

    /**
     * Adds the forward movement, strafing, and rotation together, normalizes them, and sets the motor powers appropriatly
     * @param x the strafing speed, with -1 being full speed left, and 1 being full speed right
     * @param y the forward speed, from -1 to 1
     * @param rotation the rotation speed, with -1 being full speed left, and 1 being full speed right
     */
    public void mechanumTeleOp(double x, double y, double rotation) {
        double wheelSpeeds[] = new double[4];

        wheelSpeeds[0] = x + y + rotation;
        wheelSpeeds[1] = -x + y - rotation;
        wheelSpeeds[2] = -x + y + rotation;
        wheelSpeeds[3] = x + y - rotation;

        //normalize(wheelSpeeds);
        double[] normalSpeeds = normalizeAuto(wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);

        if(speedModeOn) {
            accLeftDriveBack.setDirectPower(Range.clip((normalSpeeds[0]), -1, 1));
            accRightDriveBack.setDirectPower(Range.clip((normalSpeeds[1]), -1, 1));
            accLeftDriveFront.setDirectPower(Range.clip((normalSpeeds[2]), -1, 1));
            accRightDriveFront.setDirectPower(Range.clip((normalSpeeds[3]), -1, 1));
            //normalizeAuto(wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);
        }
        else{
            accLeftDriveBack.setTargetPower(normalSpeeds[0]);
            accRightDriveBack.setTargetPower(normalSpeeds[1]);
            accLeftDriveFront.setTargetPower(normalSpeeds[2]);
            accRightDriveFront.setTargetPower(normalSpeeds[3]);
        }
    }

    /**
     * Adjusts the motor power values to fit in the -1 to 1 range
     * @param wheelSpeeds an array with all four unadjusted motor powers
     */
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

    /**
     * Drives forward
     * @param speed the speed of movement from -1 to 1
     */
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
        //CURRENTLY ONLY USING 1 OUT OF 4 ENCODERS, COULD BE MADE MORE ACCURATE!


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
                    (leftDriveBack.isBusy()))
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
        //CURRENTLY ONLY USING 1 OUT OF 4 ENCODERS, COULD BE MADE MORE ACCURATE!
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
                    (leftDriveBack.isBusy())) {

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

    //TURNS USING IMU HEADING!!! ALTERNATIVE ONE THAT USES ENCODERS, BUT THEN IMU TO VERIFY ANGLE FOUND HERE:
    //     https://gist.github.com/lukehasawii/b17ee074bfe98d056b97725c67670539

    public void turnFor(int degrees, double speed, double timeoutS) {
        System.out.println("TURNFOR METHOD CALLED");
        degreesWanted = degrees;

        int targetDistLeft;
        int targetDistRight;
        boolean turningRight = false;
        if (opModeIsActive()) {

            if (degrees > 0) {
                turningRight = true;
            } else {
                turningRight = false;
            }

            runtime.reset();

            realAngle = getAngle();

            turn(speed, turningRight);


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (degreesWanted) != realAngle) {

                telemetry.addData("TURNING THE ROBOT to %7d DEGREES, ", "CURRENTLY AT %7d DEGREES", degreesWanted, realAngle);
                telemetry.update();
            }
            drive(0);


        }
    }


    /** Runs the proccess using encoders */
    public void runUsingEncoders(){
        System.out.println("ABOUT TO SET RUNUSINGENCODERS DIRECTLY...");

        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        System.out.println("SET IT DIRECTLY!");
    }

    /** Stops the proccess */
    public void stopDrive(){
        wheelAccelerationThread.stop();
        System.out.println("STOPDRIVE completed");
    }

    private double getAngle() {
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

    public double maxLeft, maxRight, max, ratio;

    public double[] normalizeAuto(double wheelspeeds0, double wheelspeeds1, double wheelspeeds2, double wheelspeeds3){
        /*
         * Are any of the computed wheel powers greater than 1?
         */

        if(Math.abs(wheelspeeds0) > 1
                || Math.abs(wheelspeeds1) > 1
                || Math.abs(wheelspeeds2) > 1
                || Math.abs(wheelspeeds3) > 1)
        {

            // Yeah, figure out which one

            maxLeft = Math.max(Math.abs(wheelspeeds0), Math.abs(wheelspeeds2));
            maxRight = Math.max(Math.abs(wheelspeeds1), Math.abs(wheelspeeds3));
            max = Math.max(maxLeft, maxRight);
            ratio = 1 / max; //Create a ratio to normalize them all
            double[] normalSpeeds = {(wheelspeeds0 * ratio), (wheelspeeds1 * ratio), (wheelspeeds2 * ratio), (wheelspeeds3 * ratio)};
            //leftDriveBack.setPower(wheelspeeds0 * ratio);
            //rightDriveBack.setPower(wheelspeeds1 * ratio);
            //leftDriveFront.setPower(wheelspeeds2 * ratio);
            //rightDriveFront.setPower(wheelspeeds3 * ratio);
            return normalSpeeds;
        }
        /*
         * Nothing we need to do to the raw powers
         */

        else
        {
            // leftDriveBack.setPower(wheelspeeds0);
            //  rightDriveBack.setPower(wheelspeeds1);
            // leftDriveFront.setPower(wheelspeeds2);
            // rightDriveFront.setPower(wheelspeeds3);
            double[] normalSpeedz = {(wheelspeeds0), (wheelspeeds1), (wheelspeeds2), (wheelspeeds3)};
            return normalSpeedz;
        }
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
        if(bumperNumber > 0.3){
            bumperPressed = true;
        }
        else{
            bumperPressed = false;
        }
        return bumperPressed;
    }

    public UhaulDriveTrain(LinearOpMode opMode) {
        super(opMode);
    }

}


