package org.firstinspires.ftc.teamcode.TANK;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BaseLinearOpMode;

@TeleOp(name="TankTeleOp", group="Tank")

public class TankTeleOp extends BaseLinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void run()
    {
        waitForStart();



        //robot = new Robot();
        //no not need to re-initialize robot

        runtime.reset();
        telemetry.addData("Runtime Reset", "Complete");

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            robot.teleOpActivated();

        }
    }
}
