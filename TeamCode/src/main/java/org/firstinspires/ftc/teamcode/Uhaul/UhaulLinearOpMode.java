package org.firstinspires.ftc.teamcode.Uhaul;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Uhaul.Uhaul;

/**
 * Abstract class that is run through other pseudo-opmodes.
 * @author Raw Bacon Coders
 */
public abstract class UhaulLinearOpMode extends LinearOpMode
{
    protected Uhaul robot;

    /**
     * Is called by ftc-appmaster to run the opmode
     * @throws InterruptedException
     */
    @Override
    public final void runOpMode() throws InterruptedException
    {

        robot = new Uhaul(this);
        run();

    }
    
    /**
     * Runs the entire code
     */
    public abstract void run();
    public abstract void on_init();
    public abstract void on_stop();
}
