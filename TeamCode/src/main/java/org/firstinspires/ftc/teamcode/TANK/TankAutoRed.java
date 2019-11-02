package org.firstinspires.ftc.teamcode.TANK;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TankAutoRed", group="Tank")
public class TankAutoRed extends TankLinearOpMode {
    final double DRIVE_SPEED = 0.6;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void run(){
        waitForStart();

        robot.reverseMotors();

        robot.driveForward(67.5, -DRIVE_SPEED);
        robot.strafeRight(17.5, DRIVE_SPEED);
        robot.driveForward(47.5, DRIVE_SPEED);
        robot.driveForward(10, -DRIVE_SPEED);
        robot.strafeRight(40, - DRIVE_SPEED);

    }

}
