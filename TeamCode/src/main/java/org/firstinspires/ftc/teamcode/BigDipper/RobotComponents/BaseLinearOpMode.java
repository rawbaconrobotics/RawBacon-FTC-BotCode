package org.firstinspires.ftc.teamcode.BigDipper.RobotComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.BigDipper.Robot;

public abstract class BaseLinearOpMode extends LinearOpMode
{
    protected Robot robot;
    public boolean init = true;


    @Override
    public final void runOpMode() throws InterruptedException
    {

        robot = new Robot(this);
        //robot.startup();

        on_init();
        System.out.println("Initialized robot");
        waitForStart();
        System.out.println("Play has been pressed");
        init = false;
        run();
        System.out.println("Finished running the robot");
        on_stop();
        System.out.println("Robot Stopped!");

        //robot.shutdown();

    }

    public abstract void run();
    public abstract void on_init();
    public abstract void on_stop();

}
