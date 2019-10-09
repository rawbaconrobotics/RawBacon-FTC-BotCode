package org.firstinspires.ftc.teamcode.TANK;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="TANKAutoBlue", group="TANK")
public class TANKAutoBlue extends OpMode {

    TANK tank = new TANK(hardwareMap);

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
