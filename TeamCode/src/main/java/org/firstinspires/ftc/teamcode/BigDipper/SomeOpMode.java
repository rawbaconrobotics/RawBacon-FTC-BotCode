package org.firstinspires.ftc.teamcode.BigDipper;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BDGrabber;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BDLatch;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BaseLinearOpMode;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.RobotWheels;

@TeleOp(name="OFFICIAL TeleOp", group="Big Dipper")

public class SomeOpMode extends BaseLinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    RobotWheels robotWheels;
    public BDLatch bdlatch;
    public BDGrabber bdgrabber;

    @Override
    public void on_init(){
        robot.robotWheelsTest.init();
        robot.bdlatch.init();
        robot.bdgrabber.init();
    }





    @Override
    public void run()
    {
        //robotWheels.init();
        //distanceSensor.init();
        //bdlatch.init();


        runtime.reset();
        telemetry.addData("Runtime Reset", "Complete");

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            robot.teleOpActivated();

        }
    }

    @Override
    public void on_stop() {

        robot.robotWheelsTest.stopDrive();
    }


}
