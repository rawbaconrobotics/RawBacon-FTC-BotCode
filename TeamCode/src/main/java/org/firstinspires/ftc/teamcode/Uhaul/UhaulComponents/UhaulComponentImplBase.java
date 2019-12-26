package org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.RobotComponent;

public abstract class UhaulComponentImplBase implements UhaulRobotComponent
{
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected Gamepad gamepad1;
    protected Gamepad gamepad2;
    public LinearOpMode opMode;

    public UhaulComponentImplBase(LinearOpMode opMode)
    {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        gamepad1 = opMode.gamepad1;
        gamepad2 = opMode.gamepad2;
        this.opMode = opMode;
    }

    public boolean opModeIsActive()
    {
        return opMode.opModeIsActive();
    }
}