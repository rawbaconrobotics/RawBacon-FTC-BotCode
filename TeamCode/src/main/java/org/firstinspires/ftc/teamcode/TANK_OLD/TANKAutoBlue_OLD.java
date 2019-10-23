package org.firstinspires.ftc.teamcode.TANK_OLD;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="TANKAutoBlue_OLD", group="TANK_OLD")
public class TANKAutoBlue_OLD extends OpMode {

    TANK_OLD tank = new TANK_OLD(hardwareMap);

    public void init() {
        telemetry.addData("Status", "Initialized");
        tank.drive.hwMap();
        tank.drive.setDirection();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {

    }

    @Override
    public void start() {
        tank.drive.encoderStrafeLeft(30, -0.5);
        tank.latch.moveLatch();
        tank.drive.encoderStrafeRight(30, 0.5);
        tank.latch.moveLatch();
    }

    @Override
    public void stop() {
        tank.shutdown();
    }
}
