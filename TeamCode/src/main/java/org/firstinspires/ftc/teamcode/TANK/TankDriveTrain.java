package org.firstinspires.ftc.teamcode.TANK;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


public class TankDriveTrain extends TankComponentImplBase {

    private final static String FRONTRIGHT_WHEEL_NAME = "right_drive_front";
    private final static String FRONTLEFT_WHEEL_NAME = "left_drive_front";
    private final static String BACKRIGHT_WHEEL_NAME = "right_drive_back";
    private final static String BACKLEFT_WHEEL_NAME = "left_drive_back";
    private final static int NEVEREST_ENCODER_COUNTS_PER_REVOLUTION = 1680;


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDriveBack = null;
    private DcMotor rightDriveBack = null;
    private DcMotor rightDriveFront = null;
    private DcMotor leftDriveFront = null;

    public TankDriveTrain(LinearOpMode opMode) {
        super(opMode);
    }


    @Override
    public void init() {

        leftDriveBack   = hardwareMap.dcMotor.get(BACKLEFT_WHEEL_NAME);
        rightDriveBack  = hardwareMap.dcMotor.get(BACKRIGHT_WHEEL_NAME);
        leftDriveFront  = hardwareMap.dcMotor.get(FRONTLEFT_WHEEL_NAME);
        rightDriveFront = hardwareMap.dcMotor.get(FRONTRIGHT_WHEEL_NAME);
    }

    public void wheelsTeleOp() {
        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.FORWARD);
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);

        runtime.reset();
        mechanumTeleOp(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);    // Initialize the hardware variables. Note that the strings used here as parameters
    }


    public void mechanumTeleOp(double x, double y, double rotation) {
        double wheelSpeeds[] = new double[4];

        wheelSpeeds[0] = x + y + rotation;
        wheelSpeeds[1] = -x + y - rotation;
        wheelSpeeds[2] = -x + y + rotation;
        wheelSpeeds[3] = x + y - rotation;

        normalize(wheelSpeeds);

        leftDriveBack.setPower(wheelSpeeds[0]);
        rightDriveBack.setPower(wheelSpeeds[1]);
        leftDriveFront.setPower(wheelSpeeds[2]);
        rightDriveFront.setPower(wheelSpeeds[3]);
    }


    public void encoderDrive(double distance, double speedlf, double speedlb, double speedrf, double speedrb)
    {
        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDriveFront.setTargetPosition((int) (distance * NEVEREST_ENCODER_COUNTS_PER_REVOLUTION));
        // set left motor to run to target encoder position and stop with brakes on.
        leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set right motor to run without regard to an encoder.
        rightDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Mode", "running");
        telemetry.update();

        // set left motor to run to distance encoder counts.


        // set both motors to 25% power. The movement will start.

        leftDriveFront.setPower(speedlf);
        rightDriveFront.setPower(speedrf);
        leftDriveBack.setPower(speedlb);
        rightDriveBack.setPower(speedrb);

        // wait while opmode is active and left motor is busy running to the position.

        while (opModeIsActive() && leftDriveFront.isBusy())
        {
            telemetry.addData("encoder-fwd", leftDriveFront.getCurrentPosition() + "  busy=" + leftDriveFront.isBusy());
            telemetry.update();
        }

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.

        leftDriveFront.setPower(0.0);
        rightDriveFront.setPower(0.0);
        leftDriveBack.setPower(0.0);
        rightDriveBack.setPower(0.0);
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
}