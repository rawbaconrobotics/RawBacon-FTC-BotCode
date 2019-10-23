package org.firstinspires.ftc.teamcode.TANK_OLD;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class TANKLatch_OLD {

    public OpMode OPMODE;
    public Servo TankLatch = null;
    private double TANK_LATCH_DOWN = 0;
    private double TANK_LATCH_UP = 0.2;
    private boolean TANK_LATCH_POSITION;
    private HardwareMap Mapson;
    OpMode opMode = this.OPMODE;


    public TANKLatch_OLD(HardwareMap SirMap) {
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