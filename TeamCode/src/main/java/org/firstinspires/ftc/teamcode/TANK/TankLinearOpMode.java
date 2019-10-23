package org.firstinspires.ftc.teamcode.TANK;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BigDipper.Robot;

public abstract class TankLinearOpMode extends LinearOpMode
{
    protected Robot robot;

    @Override
    public final void runOpMode() throws InterruptedException
    {

        robot = new Robot(this);
        run();

    }

    public abstract void run();
}
