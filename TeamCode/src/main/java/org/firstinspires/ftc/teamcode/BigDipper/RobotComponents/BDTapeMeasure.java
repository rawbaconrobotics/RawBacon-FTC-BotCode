package org.firstinspires.ftc.teamcode.BigDipper.RobotComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Represents the grabber
 * @author Raw Bacon Coders
 */

public class BDTapeMeasure extends RobotComponentImplBase{
    private final static String TAPE_MEASURE_NAME = "bd_tape_measure";


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private CRServo tapeMeasureServo = null;
    double measurePower = 0;
    double previousPower = 0;



    /**
     * Overrides the opMode method
     */
    public BDTapeMeasure(LinearOpMode opMode) {
        super(opMode);
    }

    /**
     * Initializes the grabber
     */
    @Override
    public void init() {
        tapeMeasureServo = hardwareMap.crservo.get(TAPE_MEASURE_NAME);
    }

    /**
     * Initializes the grabber
     */

    @Override
    public void initAutonomous() {
        tapeMeasureServo = hardwareMap.crservo.get(TAPE_MEASURE_NAME);
    }

    /**
     * Opens or closes the grabber based upon the current state it is in
     */
    public void tapeMeasureTeleOp() {

        measurePower = gamepad2.left_stick_y;
        if (opModeIsActive()) {
            if (measurePower != previousPower) {
                tapeMeasureServo.setPower(measurePower);
                previousPower = measurePower;
            }
        }
    }

}
