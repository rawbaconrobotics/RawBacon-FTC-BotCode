package org.firstinspires.ftc.teamcode.BigDipper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BaseLinearOpMode;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.RobotWheelsTest;

import static org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BDLatch.latchButton;

@TeleOp(name="OpModev1", group="Big Dipper")

public class SomeOpMode extends BaseLinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    RobotWheelsTest robotWheelsTest;

    @Override
    public void run()
    {
        waitForStart();



        //robot = new Tank();
        //no not need to re-initialize robot

        runtime.reset();
        telemetry.addData("Runtime Reset", "Complete");

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            robot.teleOpActivated();

        }
        robotWheelsTest.stopDrive();
    }


}
