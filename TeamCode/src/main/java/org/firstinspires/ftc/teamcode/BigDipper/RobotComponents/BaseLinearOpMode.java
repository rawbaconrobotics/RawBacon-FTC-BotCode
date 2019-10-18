package org.firstinspires.ftc.teamcode.BigDipper.RobotComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.BigDipper.Robot;

public abstract class BaseLinearOpMode extends LinearOpMode {
    protected Robot robot;

    @Override
    public final void runOpMode(LinearOpMode opMode) throws InterruptedException
    {
        robot = new Robot(opMode);
        run(opMode);
    }

    public abstract void run(LinearOpMode opMode);
    {

    }

}
