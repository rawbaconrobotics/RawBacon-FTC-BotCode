package org.firstinspires.ftc.teamcode.TANK;


import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp(name = "TANKLatch",group = "TANK")
public class TANKLatch {

    public OpMode OPMODE;
    private Servo TankLatch;
    private float TANK_LATCH_DOWN = 30;
    private float TANK_LATCH_UP = 0;
    private boolean TANK_LATCH_POSITION;
    private HardwareMap Mapson;
    OpMode opMode = this.OPMODE;


    public TANKLatch(HardwareMap SirMap) {
        Mapson = SirMap;
        TankLatch = Mapson.servo.get("tank_latch");
        TankLatch.setPosition(TANK_LATCH_UP);
        TANK_LATCH_POSITION = true;
    }

    public void moveLatch() {
        if (TANK_LATCH_POSITION) {
            TankLatch.setPosition(TANK_LATCH_DOWN);
            TANK_LATCH_POSITION = false;
            opMode.telemetry.addData("latch position:","down");
        } else {
            TankLatch.setPosition(TANK_LATCH_UP);
            TANK_LATCH_POSITION = true;
            opMode.telemetry.addData("latch position:","up");
        }
    }
}