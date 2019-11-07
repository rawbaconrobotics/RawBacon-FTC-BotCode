package org.firstinspires.ftc.teamcode.Clyde;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;

public class FergieArm implements HardwareHelper{
    private Servo rightClaw; //Declare right servo of claw
    private Servo leftClaw;  //Declare left servo of claw
    private DcMotor arm;     //Declare arm servo


    // *** Change these assignments, future Max ***
    private final String RIGHT_CLAW_NAME = "right_claw";
    private final String LEFT_CLAW_NAME = "left_claw";
    private final String ARM_MOTOR_NAME = "arm_motor";
    private final double RIGHT_OPEN_POSITION = 3;
    private final double LEFT_OPEN_POSITION = 3;
    private final double LEFT_CLOSED_POSITION = 6;
    private final double RIGHT_CLOSED_POSITION = 6;
    //Both of these are encoder counts
    private final double ARM_LOWER_BOUND = 100;
    private final double ARM_UPPER_BOUND = 600;
    //Force the arm to move at half speed
    private final double ARM_SPEED_MULT = 0.5;

    //Map all of the motors/servos
    public void init(HardwareMap mappy){
        rightClaw = mappy.get(Servo.class, RIGHT_CLAW_NAME);
        leftClaw = mappy.get(Servo.class, LEFT_CLAW_NAME);
        arm = mappy.get(DcMotor.class, ARM_MOTOR_NAME);
        rightClaw.setPosition(0);
        leftClaw.setPosition(0);
        arm.setPower(0);
    }

    //Set the speed to go up, stop if higher than should be
    public void moveUp(double speed){
        speed *= ARM_SPEED_MULT;
        arm.setPower(speed);
        if (arm.getCurrentPosition() >= ARM_UPPER_BOUND) {
            arm.setPower(0);
        }
    }
    //Set the speed to go down, stop if lower than should be
    public void moveDown(double speed){
        speed  *= ARM_SPEED_MULT;
        arm.setPower(-speed);
        if (arm.getCurrentPosition() <= ARM_LOWER_BOUND) {
            arm.setPower(0);
        }
    }
    //Acts as a mediator for moveUp and moveDown
    public void moveArm(double speed) {
        if (speed > 0){
            moveUp(speed);
        }
        else {
            if (speed == 0){
                arm.setPower(0);
            }
            else {
                moveDown(-speed);
            }
        }
    }

    public void resetEncoders(){
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //Opens the claw
    public void openClaw(){
        rightClaw.setPosition(RIGHT_OPEN_POSITION);
        leftClaw.setPosition(LEFT_OPEN_POSITION);
    }

    //Closes the claw
    public void closeClaw(){
        rightClaw.setPosition(RIGHT_CLOSED_POSITION);
        leftClaw.setPosition(LEFT_CLOSED_POSITION);
    }
}
