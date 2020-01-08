package org.firstinspires.ftc.teamcode.Uhaul;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A linear opmode that activates the teleop stored in the {@link Uhaul} class
 * @author Luke Aschenbrener
 */
@TeleOp(name="UhaulTeleOp", group="Uhaul")
public class  UhaulTeleOp extends UhaulLinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void on_init() {
        robot.drive.init();
        robot.latch.init();
        robot.grabber.init();
        robot.arm.init();
        robot.intake.init();
        robot.lift.init();
    }

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

    @Override
    public void on_stop() {
        robot.drive.stopDrive();
    }
}


