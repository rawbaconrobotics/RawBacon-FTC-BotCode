package org.firstinspires.ftc.teamcode.BigDipper.RobotComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Adds methods relevent to the components
 * @author Raw Bacon Coders
 */
public abstract class RobotComponentImplBase implements RobotComponent
{
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected Gamepad gamepad1;
    protected Gamepad gamepad2;
    public LinearOpMode opMode;

    /**
     * Constructor
     * @param opMode The opmode in use. Use keyword this.
     */
    public RobotComponentImplBase(LinearOpMode opMode)
    {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        gamepad1 = opMode.gamepad1;
        gamepad2 = opMode.gamepad2;
        this.opMode = opMode;
    }

    /**
     * @returns Whether the opmode is active
     */
    public boolean opModeIsActive()
    {
        return opMode.opModeIsActive();
    }
    public boolean isStopRequested(){return opMode.isStopRequested();}

}
