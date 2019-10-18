package org.firstinspires.ftc.teamcode.BigDipper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BaseLinearOpMode;

import static org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BDLatch.latchButton;

@TeleOp(name="OpModev1", group="Big Dipper")

public class SomeOpMode extends BaseLinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void run(LinearOpMode opMode) {
        waitForStart();
        //robot = new Robot();
        //no not need to re-initialize robot

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.teleOpActivated(opMode);
            latchButton = true;
        }
    }





    //We don't use these down here, they are overridden in the BaseLinearOpMode


    public void runOpMode() throws InterruptedException {

    }

    public void runOpMode(OpMode opMode) throws InterruptedException {

    }








}
