package org.firstinspires.ftc.teamcode.BigDipper.RobotComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.BigDipper.Robot;

public abstract class BaseLinearOpMode extends LinearOpMode
{
    protected Robot robot;


    @Override
    public final void runOpMode() throws InterruptedException
    {

        robot = new Robot(this);
        //robot.startup();
        on_init();
        waitForStart();
        run();
        while(opModeIsActive() && !isStopRequested()) {
            idle();
        }
        on_stop();
        //robot.shutdown();

    }

    public abstract void run();
    public abstract void on_init();
    public abstract void on_stop();

}
