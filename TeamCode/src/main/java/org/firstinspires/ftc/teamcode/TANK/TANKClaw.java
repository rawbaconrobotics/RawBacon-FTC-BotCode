package org.firstinspires.ftc.teamcode.TANK;


import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Disabled
public class TANKClaw {
    public OpMode OPMODE;
    private DcMotor TANKArm;
    private Servo TankClaw;
    public float TANK_CLAW_OPEN = 30;
    public float TANK_CLAW_CLOSED = 0;
    public boolean TANK_CLAW_POSITION; //true is closed, false is open
    public double TANK_ARM_SPEED;
    public float armspeed = 1;
    private HardwareMap mappy;
    OpMode opMode = this.OPMODE;

    public TANKClaw(HardwareMap map) {
        mappy = map;
        TANKArm = mappy.dcMotor.get("tank_arm");
        TankClaw = mappy.servo.get("tank_claw");
        TANKArm.setPower(0);
        TankClaw.setPosition(TANK_CLAW_CLOSED);
        TANK_CLAW_POSITION = true;
        TANK_ARM_SPEED = 1;
    }

    public void TANKMoveClaw() {
        if (TANK_CLAW_POSITION = true) {
            TankClaw.setPosition(TANK_CLAW_OPEN);
            TANK_CLAW_POSITION = false;
        } else {
            TankClaw.setPosition(TANK_CLAW_CLOSED);
            TANK_CLAW_POSITION = true;
        }
    }

    public void TANKMoveArm(){
        TANKArm.setPower(opMode.gamepad2.right_stick_y*TANK_ARM_SPEED);
    }

    public void TANKChangeArmSpeed() {
        if (TANK_ARM_SPEED == 1){
            TANK_ARM_SPEED = 0.5;
        }
        else {
            TANK_ARM_SPEED = 1;
        }
    }

    public void TANKStopClaw () {
        TANKArm.setPower(0);
    }
}




