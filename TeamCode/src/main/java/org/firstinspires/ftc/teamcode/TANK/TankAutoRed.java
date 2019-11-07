package org.firstinspires.ftc.teamcode.TANK;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TankAutoRed", group="Tank")
public class TankAutoRed extends TankLinearOpMode {
    final double driveSpeed = 0.6;
    final double latchClosed = 0.2;
    final double latchOpen = 0.9;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void run(){

        robot.setUpAuto();
        waitForStart();

        robot.reverseMotors();
        robot.driveBackward(31.5, driveSpeed, 10);
        sleep(500);
        robot.autoLatch(latchClosed, latchOpen);
        sleep(500);
        robot.driveForward(31.5, driveSpeed, 10);
        sleep(500);
        robot.autoLatch(latchOpen, latchClosed);
        sleep(500);
        robot.strafeLeft(31.5, driveSpeed, 10);
        sleep(500);







    }

}
