package org.firstinspires.ftc.teamcode.RobotComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

public abstract class BaseLinearOpMode extends LinearOpMode {
    protected Robot robot;

    @Override
    public final void runOpMode() throws InterruptedException
    {
        robot = new Robot();
        robot.init(hardwareMap, true);
        run();
    }

    public abstract void run();
    {

    }

}
