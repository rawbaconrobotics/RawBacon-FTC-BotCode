package org.firstinspires.ftc.teamcode.TANK;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TankAutoBlue", group="Tank")
public class TankAutoBlue extends TankLinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void run(){
        waitForStart();

        robot.driveForward(75, -0.6);
        robot.turnLeft(90, 0.6);
        robot.driveForward(21.25, 0.6);
        robot.turnRight(90, 0.6);
        robot.driveForward(55,0.6);
        robot.driveForward(5, -0.6);
        robot.turnRight(90, 0.6);
        robot.driveForward(35, 0.6);
    }

}
