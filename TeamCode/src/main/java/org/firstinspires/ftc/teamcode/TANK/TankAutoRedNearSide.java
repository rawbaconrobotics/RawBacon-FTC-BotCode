package org.firstinspires.ftc.teamcode.TANK;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TankAutoRedNearSide", group="Tank")
public class TankAutoRedNearSide extends TankLinearOpMode {
    final double driveSpeed = 0.4;
    final double latchClosed = 0.2;
    final double latchOpen = 0.9;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void run(){


        waitForStart();
        robot.driveByTime(-0.4, 0.4, 0.4, -0.4, 0.35);
        robot.driveByTime(-0.4, -0.4, -0.4, -0.4, 1.4);
        sleep(500);
        robot.autoLatch(latchClosed, latchOpen);
        sleep(1000);
        robot.driveByTime(0.6, 0.62, 0.6, 0.62, 2.5);
        sleep(500);
        robot.autoLatch(latchOpen, latchClosed);
        sleep(1000);
        robot.driveByTime(0.4, -0.4, -0.4, 0.4, 2.5);





    }

}
