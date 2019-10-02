package org.firstinspires.ftc.teamcode.TANK;


import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name = "TANKClaw",group = "TANK")
public class TANKClaw {
    public OpMode OPMODE;
    public DcMotor TANKArm;
    public Servo TankClaw;
    public float TANK_CLAW_OPEN = 30;
    public float TANK_CLAW_CLOSED = 0;
    public boolean TANK_CLAW_POSITION; //true is closed, false is open
    public float armspeed = 1;
    private HardwareMap Mapson;
    OpMode opMode = this.OPMODE;

    public TANKClaw(HardwareMap SirMap) {
        Mapson = SirMap;
        TANKArm = Mapson.dcMotor.get("tank_arm");
        TankClaw = Mapson.servo.get("tank_claw");
        TANKArm.setPower(0);
        TankClaw.setPosition(TANK_CLAW_CLOSED);
        TANK_CLAW_POSITION = true;
    }

    public void MoveClaw() {
        if (TANK_CLAW_POSITION = true) {
            TankClaw.setPosition(TANK_CLAW_OPEN);
            TANK_CLAW_POSITION = false;
        } else {
            TankClaw.setPosition(TANK_CLAW_CLOSED);
            TANK_CLAW_POSITION = true;
        }
    }

    public void TANKMoveArm(){

        TANKArm.setPower(opMode.gamepad2.right_stick_y*armspeed);

    }


}




