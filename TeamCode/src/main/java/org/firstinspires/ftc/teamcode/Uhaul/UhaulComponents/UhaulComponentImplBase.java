package org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.RobotComponent;


/**
 * @author Raw Bacon Coders
 * Establishes a component base for the Uhaul robot
 */
public abstract class UhaulComponentImplBase implements UhaulRobotComponent
{
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected Gamepad gamepad1;
    protected Gamepad gamepad2;
    public LinearOpMode opMode;

    /** 
     * Constructor
     * @param opMode This parameter takes in a LinearOpMode as the variable opMode.
     *
     */
    public UhaulComponentImplBase(LinearOpMode opMode)
    {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        gamepad1 = opMode.gamepad1;
        gamepad2 = opMode.gamepad2;
        this.opMode = opMode;
    }

    /** Returns whether the opmode is active or not */
    public boolean opModeIsActive()
    {
        return opMode.opModeIsActive();
    }
}
