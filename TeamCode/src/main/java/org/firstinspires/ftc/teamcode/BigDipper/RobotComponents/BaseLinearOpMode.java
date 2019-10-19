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
        telemetry.addData("e", "I");

        robot = new Robot(this);
        run();

    }

    public abstract void run();
}
