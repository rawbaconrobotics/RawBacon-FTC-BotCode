package org.firstinspires.ftc.teamcode.Uhaul.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Uhaul.UhaulLinearOpMode;


/**
 * @author Raw Bacon Coders
 * This class tests the Uhaul Life Encoder
 */
@TeleOp(name="Uhaul Lift Encoder Test", group="Uhaul")
public class UhaulLiftEncoderValueTest extends UhaulLinearOpMode {


    
    /**
    * Initializes the code
    */
    @Override
    public void on_init() {
        robot.lift.initForTesting();
    }
    /**
     * Runs the code and updates the telemetry
     */
    @Override
    public void run() {
        while(opModeIsActive()){
            telemetry.addData("Lift One Position: ", robot.lift.uhaulLift.getCurrentPosition());
            telemetry.addData("Lift Two Position: ", robot.lift.uhaulLiftTwo.getCurrentPosition());
            telemetry.update();

            robot.lift.uhaulLiftTwo.setPower(gamepad1.right_stick_y /4);
            robot.lift.uhaulLift.setPower(gamepad1.right_stick_y/4);
        }
    }

    /**
    * Stops the process 
    */
    @Override
    public void on_stop() {

    }
}
