package org.firstinspires.ftc.teamcode.TANK;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


public class TankDriveTrain extends TankComponentImplBase {
    public void initAutonomous(){}

    private final static String FRONTRIGHT_WHEEL_NAME = "right_drive_front";
    private final static String FRONTLEFT_WHEEL_NAME = "left_drive_front";
    private final static String BACKRIGHT_WHEEL_NAME = "right_drive_back";
    private final static String BACKLEFT_WHEEL_NAME = "left_drive_back";
    private static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // eg: Neverest Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftDriveBack = null;
    public DcMotor rightDriveBack = null;
    public DcMotor rightDriveFront = null;
    public DcMotor leftDriveFront = null;

    public TankDriveTrain(LinearOpMode opMode) {
        super(opMode);
    }


    @Override
    public void init() {

        leftDriveBack   = hardwareMap.dcMotor.get(BACKLEFT_WHEEL_NAME);
        rightDriveBack  = hardwareMap.dcMotor.get(BACKRIGHT_WHEEL_NAME);
        leftDriveFront  = hardwareMap.dcMotor.get(FRONTLEFT_WHEEL_NAME);
        rightDriveFront = hardwareMap.dcMotor.get(FRONTRIGHT_WHEEL_NAME);

        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);
    }

    public void wheelsTeleOp() {
        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);

        runtime.reset();
        mechanumTeleOp(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);    // Initialize the hardware variables. Note that the strings used here as parameters
    }


    public void mechanumTeleOp(double x, double y, double rotation) {
        double wheelSpeeds[] = new double[4];

        wheelSpeeds[0] = x + y + rotation;
        wheelSpeeds[1] = -x + y - rotation;
        wheelSpeeds[2] = -x + y + rotation;
        wheelSpeeds[3] = x + y - rotation;

        //normalize(wheelSpeeds);

        leftDriveBack.setPower(wheelSpeeds[0]);
        rightDriveBack.setPower(wheelSpeeds[1]);
        leftDriveFront.setPower(wheelSpeeds[2]);
        rightDriveFront.setPower(wheelSpeeds[3]);
    }

    public void reverseWheels(){
        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);
    }
    public void runMotors(double lfSpeed, double rfSpeed, double lbSpeed, double rbSpeed, double time){
        leftDriveFront.setPower(lfSpeed);
        rightDriveFront.setPower(rfSpeed);
        leftDriveBack.setPower(lbSpeed);
        rightDriveBack.setPower(rbSpeed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        leftDriveFront.setPower(0);
        rightDriveFront.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);
    }

    public void setUpAuto() {
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                leftDriveFront.getCurrentPosition(),
                rightDriveFront.getCurrentPosition(),
                leftDriveBack.getCurrentPosition(),
                rightDriveBack.getCurrentPosition());
        telemetry.update();
    }
    public void encoderDrive(double speed,
                             double lfInches, double rfInches,
                             double lbInches, double rbInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftDriveFront.getCurrentPosition() + (int)(lfInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftDriveBack.getCurrentPosition() + (int)(rfInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightDriveFront.getCurrentPosition() + (int)(lbInches * COUNTS_PER_INCH);
            newRightBackTarget = rightDriveBack.getCurrentPosition() + (int)(rbInches * COUNTS_PER_INCH);

            leftDriveFront.setTargetPosition(newLeftFrontTarget);
            leftDriveBack.setTargetPosition(newLeftBackTarget);
            rightDriveFront.setTargetPosition(newRightFrontTarget);
            rightDriveBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDriveFront.setPower(speed);
            rightDriveFront.setPower(speed);
            rightDriveBack.setPower(speed);
            leftDriveBack.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDriveFront.isBusy() && rightDriveFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftDriveFront.getCurrentPosition(),
                        rightDriveFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDriveFront.setPower(0);
            rightDriveFront.setPower(0);
            leftDriveBack.setPower(0);
            rightDriveBack.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


    /*private void normalize(double[] wheelSpeeds) {
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
        }*/
    }
