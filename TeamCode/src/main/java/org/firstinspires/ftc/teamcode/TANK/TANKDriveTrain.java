package org.firstinspires.ftc.teamcode.TANK;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TANKDriveTrain{
    public OpMode opmode;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public HardwareMap mappy;
    final private double STRAIGHT = 1440/(4*Math.PI);
    final private double ROT = STRAIGHT*16.5*Math.PI/360;

    public TANKDriveTrain(HardwareMap map){
        mappy = map;
    }
    public void hwMap(){
        rightBackDrive  = mappy.get(DcMotor.class,"right_back_drive");
        rightFrontDrive = mappy.get(DcMotor.class,"right_front_drive");
        leftBackDrive   = mappy.get(DcMotor.class,"left_back_drive");
        leftFrontDrive  = mappy.get(DcMotor.class,"left_front_drive");
        OpMode opmode = this.opmode;
    }
    public void setDirection(){
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
    }
    public void fourWheelDrive(double LEFT_FRONT,
                               double RIGHT_FRONT,
                               double LEFT_BACK,
                               double RIGHT_BACK){
        leftBackDrive.setPower(LEFT_BACK);
        rightBackDrive.setPower(RIGHT_BACK);
        leftFrontDrive.setPower(LEFT_FRONT);
        rightFrontDrive.setPower(RIGHT_FRONT);
    }
    public void teleOpDrive(){
        fourWheelDrive(opmode.gamepad1.left_stick_y,
                       opmode.gamepad1.right_stick_y,
                       opmode.gamepad1.left_stick_y,
                       opmode.gamepad1.right_stick_y);
    }
    public void strafeRightDrive(double speed){
        fourWheelDrive(-speed, speed, speed, -speed);
    }
    public void strafeLeftDrive(double speed){
        strafeRightDrive(-speed);
    }
    public void halt(){
        fourWheelDrive(0, 0, 0, 0);
    }
    public void encoderDrive(double distance, double speed)
    {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set left motor to run to target encoder position and stop with brakes on.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set right motor to run without regard to an encoder.
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // set left motor to run to distance encoder counts.

        leftFrontDrive.setTargetPosition((int) (distance*STRAIGHT));

        // set both motors to 25% power. The movement will start.

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        // wait while opmode is active and left motor is busy running to the position.

        while (leftFrontDrive.isBusy());

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.

        leftFrontDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
    }
    public void encoderStrafeRight(double distance, double speed)
    {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set left motor to run to target encoder position and stop with brakes on.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set right motor to run without regard to an encoder.
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // set left motor to run to distance encoder counts.

        leftFrontDrive.setTargetPosition((int) (distance*STRAIGHT));

        // set both motors to 25% power. The movement will start.

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(-speed);
        leftBackDrive.setPower(-speed);
        rightBackDrive.setPower(speed);

        // wait while opmode is active and left motor is busy running to the position.

        while (leftFrontDrive.isBusy());

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    public void encoderStrafeLeft(double distance, double speed){
        encoderStrafeRight(distance, -speed);
    }
    public void encoderTurnRight(double distance, double speed){

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set left motor to run to target encoder position and stop with brakes on.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set right motor to run without regard to an encoder.
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // set left motor to run to distance encoder counts.

        leftFrontDrive.setTargetPosition((int) (distance*ROT));

        // set both motors to 25% power. The movement will start.

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(-speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(-speed);

        // wait while opmode is active and left motor is busy running to the position.

        while (leftFrontDrive.isBusy());

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.

        leftFrontDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
    }
    public void encoderTurnLeft(double distance, double speed){
        encoderTurnRight(distance, -speed);
    }
}
