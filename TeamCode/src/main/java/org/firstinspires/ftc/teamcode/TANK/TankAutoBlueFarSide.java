package org.firstinspires.ftc.teamcode.TANK;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TankAutoBlueFarSide", group="Tank")
public class TankAutoBlueFarSide extends TankLinearOpMode {
    final double driveSpeed = 0.4;
    final double latchClosed = 0.2;
    final double latchOpen = 0.9;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void run(){

        //robot.setUpAuto();
        robot.reverseMotors();
        waitForStart();

        robot.driveByTime(-0.4, -0.4, -0.4, -0.4, 2);
        robot.driveByTime(0.4, -0.4, -0.4, 0.4, 0.5);
        robot.driveByTime(-0.4, 0.4, 0.4, -0.4, 0.2);
        robot.driveByTime(0.4, 0.4, 0.4, 0.4, 1);
        robot.driveByTime(0.4, -0.4, -0.4, 0.4, 1.5);
        robot.driveByTime(-0.4, -0.4, -0.4, -0.4, 0.2);
        sleep(500);
        robot.autoLatch(latchClosed, latchOpen);
        sleep(1000);
        robot.driveByTime(0.6, 0.6, 0.6, 0.6, 2.5);
        sleep(500);
        robot.autoLatch(latchOpen, latchClosed);
        sleep(1000);

        robot.driveByTime(-0.4, 0.4, 0.4, -0.4, 1.5);
        robot.driveByTime(-0.4, -0.4, -0.4, -0.4, 1);
        robot.driveByTime(-0.4, 0.4, 0.4, -0.4, 1);


    }
}