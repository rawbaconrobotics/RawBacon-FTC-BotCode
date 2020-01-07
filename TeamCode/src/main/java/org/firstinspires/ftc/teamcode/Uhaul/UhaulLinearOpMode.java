package org.firstinspires.ftc.teamcode.Uhaul;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Uhaul.Uhaul;

public abstract class UhaulLinearOpMode extends LinearOpMode
{
    protected Uhaul robot;

    @Override
    public final void runOpMode() throws InterruptedException
    {

        robot = new Uhaul(this);
        run();

    }

    public abstract void run();
    public abstract void on_init();
    public abstract void on_stop();
}
