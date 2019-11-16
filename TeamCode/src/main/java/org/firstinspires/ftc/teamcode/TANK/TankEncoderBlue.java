  package org.firstinspires.ftc.teamcode.TANK;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TankEncoderBlue", group="Tank")
public class  TankEncoderBlue extends TankLinearOpMode {
    final double DRIVE_SPEED = 0.4;
    final double LATCH_CLOSED = 0.2;
    final double LATCH_OPEN = 0.9;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void run(){
        robot.setUpAuto();
        //strafe right 25inches to wall
       robot.encoderDrive(25, DRIVE_SPEED * 1, DRIVE_SPEED * -1, DRIVE_SPEED * -1, DRIVE_SPEED * 1, 5);
       sleep(500);
       //strafe left 2 inches to align with the foundation
       robot.encoderDrive(4, DRIVE_SPEED * -1, DRIVE_SPEED * 1, DRIVE_SPEED * 1, DRIVE_SPEED * -1, 5);
       sleep(500);
       //drive backwards to foundation
       robot.encoderDrive(31.5, DRIVE_SPEED * -1, DRIVE_SPEED * -1, DRIVE_SPEED * -1, DRIVE_SPEED * -1, 5);
       sleep(500);
       //latch onto the foundation
       robot.autoLatch(LATCH_CLOSED, LATCH_OPEN);
       sleep(500);
       //turn, pulling the foundation into the building site
       robot.encoderDrive(15, DRIVE_SPEED * 1, DRIVE_SPEED * 1.5, DRIVE_SPEED * 1, DRIVE_SPEED * 1.5,5);
       sleep(500);
       robot.encoderDrive(20, DRIVE_SPEED * 1, DRIVE_SPEED * 1, DRIVE_SPEED * 1, DRIVE_SPEED * 1, 5);
    }
}