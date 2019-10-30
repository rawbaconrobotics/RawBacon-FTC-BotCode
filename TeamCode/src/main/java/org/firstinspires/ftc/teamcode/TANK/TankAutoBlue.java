package org.firstinspires.ftc.teamcode.TANK;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TankAutoBlue", group="Tank")
public class TankAutoBlue extends TankLinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void run(){
        waitForStart();

        robot.driveForward(70, -0.6);
        robot.strafeRight(15, 0.6);
        robot.driveForward(50,0.6);
        robot.strafeRight(33, -0.6);
    }

}
