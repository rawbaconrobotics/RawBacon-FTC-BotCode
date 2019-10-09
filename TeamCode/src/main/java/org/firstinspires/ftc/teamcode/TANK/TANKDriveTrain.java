package org.firstinspires.ftc.teamcode.TANK;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TANKDriveTrain{
    public OpMode opmode;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private HardwareMap mappy;

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
        fourWheelDrive(speed,
                       speed,
                       speed,
                       -speed);
    }
    public void strafeLeftDrive(double speed){
        strafeRightDrive(-speed);
    }
    public void halt(){
        fourWheelDrive(0, 0, 0, 0);
    }
}
