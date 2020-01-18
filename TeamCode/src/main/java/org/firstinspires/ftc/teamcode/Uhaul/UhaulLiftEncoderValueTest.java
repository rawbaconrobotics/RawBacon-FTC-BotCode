package org.firstinspires.ftc.teamcode.Uhaul;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Uhaul Lift Encoder Test", group="Uhaul")
public class UhaulLiftEncoderValueTest extends UhaulLinearOpMode {


    @Override
    public void on_init() {
        robot.lift.initForTesting();
    }
    @Override
    public void run() {
        while(opModeIsActive()){
            telemetry.addData("Lift One Position: ", robot.lift.uhaulLift.getCurrentPosition());
            telemetry.addData("Lift Two Position: ", robot.lift.uhaulLiftTwo.getCurrentPosition());
            telemetry.update();
        }
    }

    @Override
    public void on_stop() {

    }
}
