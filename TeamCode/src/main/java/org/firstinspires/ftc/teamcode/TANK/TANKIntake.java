package org.firstinspires.ftc.teamcode.TANK;
import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TANKIntake {
    public OpMode OPMODE;
    private Servo TANKIntakeLeft;
    private Servo TANKIntakeRight;
    private DcMotor TANKLift;
    private boolean intakeOn;
    private double TANK_LIFT_SPEED;
    private HardwareMap mappy;
    OpMode opMode = this.OPMODE;

    public TANKIntake(HardwareMap map) {
        mappy = map;
        TANKIntakeLeft = mappy.servo.get("tank_left_intake");
        TANKIntakeRight = mappy.servo.get("tank_right_intake");
        TANKLift = mappy.dcMotor.get("tank_scissor_lift");
        TANKIntakeLeft.setPosition(0.5);
        TANKIntakeRight.setPosition(0.5);
        TANKLift.setPower(0);
        TANK_LIFT_SPEED = 1;
    }

    public void TANKIntake(boolean power){
        if (power) {
            TANKIntakeLeft.setPosition(1);
            TANKIntakeRight.setPosition(1);
                    
        }
        else{
            TANKIntakeLeft.setPosition(0.5);
            TANKIntakeRight.setPosition(0.5);
        }
        intakeOn = power;
    }
    public boolean getServoPower(){
        return intakeOn;
    }
    public void TANKMoveLift (){
        TANKLift.setPower(opMode.gamepad2.right_stick_y*TANK_LIFT_SPEED);
    }
    public void TANKChangeLiftSpeed(double speed) {
        TANK_LIFT_SPEED = speed;
    }
    public void shutdown(){
        TANKIntake(false);
        TANK_LIFT_SPEED = 0;
        TANKLift.setPower(0);
    }
}