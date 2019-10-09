package org.firstinspires.ftc.teamcode.BigDipper;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BaseLinearOpMode;

import static org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BDLatch.latchButton;

@TeleOp(name="OpModev1", group="Big Dipper")

public class SomeOpMode extends BaseLinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void run() {
        waitForStart();
        //robot = new Robot();
        //no not need to re-initialize robot

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.teleOpActivated();
            latchButton = true;
        }
    }
}
