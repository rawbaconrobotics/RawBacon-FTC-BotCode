package org.firstinspires.ftc.teamcode.Clyde_OLD.MaxHerrera.MHSkyStone;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FergieArm {
    private Servo rightClaw; //Declare right servo of claw
    private Servo leftClaw;  //Declare left servo of claw
    private DcMotor arm;     //Declare arm servo


    // *** Change these assignments, future Max ***
    private final String RIGHT_CLAW_NAME = "TEMP1";
    private final String LEFT_CLAW_NAME = "TEMP2";
    private final String ARM_MOTOR_NAME = "TEMP3";
    private final double RIGHT_OPEN_POSITION = 3;
    private final double LEFT_OPEN_POSITION = 3;
    private final double LEFT_CLOSED_POSITION = 3;
    private final double RIGHT_CLOSED_POSITION = 3;
    //Both of these are encoder counts
    private final double ARM_LOWER_BOUND = 3;
    private final double ARM_UPPER_BOUND = 3;

    //Class constructor
    public FergieArm(HardwareMap mappy){
        rightClaw = mappy.get(Servo.class, RIGHT_CLAW_NAME);
        leftClaw = mappy.get(Servo.class, LEFT_CLAW_NAME);
        arm = mappy.get(DcMotor.class, ARM_MOTOR_NAME);
        rightClaw.setPosition(0);
        leftClaw.setPosition(0);
    }

    //Set the speed to go up, stop if higher than it should be
    public void moveUp(double speed){
        arm.setPower(speed);
        if (arm.getCurrentPosition() >= ARM_UPPER_BOUND) {
            arm.setPower(0);
            return;
        }
    }
    //Set the speed to go down, stop if lower than it should be
    public void moveDown(double speed){
        arm.setPower(-speed);
        if (arm.getCurrentPosition() <= ARM_LOWER_BOUND) {
            arm.setPower(0);
            return;
        }
    }

    //Opens the claw
    public void openClaw(){
        rightClaw.setPosition(RIGHT_OPEN_POSITION);
        leftClaw.setPosition(LEFT_OPEN_POSITION);
    }

    //Closes the claw
    public void closeClaw(){  //CHANGE
        rightClaw.setPosition(RIGHT_CLOSED_POSITION);
        leftClaw.setPosition(LEFT_CLOSED_POSITION);
    }
}
