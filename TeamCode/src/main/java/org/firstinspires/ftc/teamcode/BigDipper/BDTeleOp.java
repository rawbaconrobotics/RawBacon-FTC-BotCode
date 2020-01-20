package org.firstinspires.ftc.teamcode.BigDipper;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BaseLinearOpMode;


/**
 * The teleop 
 * @auther Raw Bacon Coders
 */


@TeleOp(name="OFFICIAL TeleOp", group="Big Dipper")

public class BDTeleOp extends BaseLinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    
    /**
     * Initializes the teleop
     */
    @Override
    public void on_init() {
        robot.bddrivetrain.init();
        robot.bdlatch.init();
        robot.bdgrabber.init();
        robot.bdcapstone.init();
        robot.bdtapemeasure.init();
    }
    
    /**
     * Activates the teleop 
     */

    @Override
    public void run() {
        runtime.reset();
        telemetry.addData("Runtime Reset", "Complete");
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()){
            robot.teleOpActivated();
        }
    }
    
    /**
     * Stops the teleop
     */

    @Override
    public void on_stop() {
        robot.bddrivetrain.stopDrive();
    }
}
