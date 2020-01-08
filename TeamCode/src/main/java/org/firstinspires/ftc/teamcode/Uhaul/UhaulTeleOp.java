package org.firstinspires.ftc.teamcode.Uhaul;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A linear opmode that activates the teleop stored in the {@link #Uhaul} class
 * @author Luke Aschenbrener
 */
@TeleOp(name="UhaulTeleOp", group="Uhaul")
public class  UhaulTeleOp extends UhaulLinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    /**
     * Activates the teleop
     */
    @Override
    public void run()
    {
        waitForStart();

        runtime.reset();
        telemetry.addData("Runtime Reset", "Complete");

        while (opModeIsActive())
        {
            robot.teleOpActivated();

        }
    }
}
