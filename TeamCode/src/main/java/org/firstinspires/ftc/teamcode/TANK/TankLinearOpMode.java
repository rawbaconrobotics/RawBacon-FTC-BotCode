package org.firstinspires.ftc.teamcode.TANK;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TANK.Tank;

public abstract class TankLinearOpMode extends LinearOpMode
{
    protected Tank robot;

    @Override
    public final void runOpMode() throws InterruptedException
    {

        robot = new Tank(this);
        run();

    }

    public abstract void run();
}
