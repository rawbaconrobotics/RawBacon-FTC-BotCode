package org.firstinspires.ftc.teamcode.TANK;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.Drivetrain;

@Autonomous(name="TankEncoderTest", group="Tank")
public class TankEncoderTest extends TankLinearOpMode {
    final double DRIVE_SPEED = 0.4;
    final double LATCH_CLOSED = 0.2;
    final double LATCH_OPEN = 0.9;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void run() {
    robot.encoderDrive(5, DRIVE_SPEED, DRIVE_SPEED, DRIVE_SPEED, DRIVE_SPEED, 15);
    }
}