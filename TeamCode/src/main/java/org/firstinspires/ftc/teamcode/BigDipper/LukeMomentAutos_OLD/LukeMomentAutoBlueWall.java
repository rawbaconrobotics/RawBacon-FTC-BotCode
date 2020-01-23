package org.firstinspires.ftc.teamcode.BigDipper.LukeMomentAutos_OLD;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BaseLinearOpMode;


/**
 * Autonomous code to score the foundation and park under the bridge
 * near the wall when the robot is on the blue alliance.
 * @author Raw Bacon Coders
 */

@Disabled
@Autonomous(name = "LukeMomentAutoBlueWall", group = "Big Dipper")

public class LukeMomentAutoBlueWall extends BaseLinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public static double FIRSTdriveinches = -39;
    public static double SECONDstrafeinches = 6;
    public static double THIRDstrafeinches = 4;
    public static double FOURTHdriveinches = 25;
    public static double FIFTHstrafeinches = 22;
    public static double SIXTHdriveinches = -16.5;
    public static double SEVENTHdriveinches = 29;
    public static double NINTHstrafeinches = 47;
    
    /**
     * Initializes the components the robot uses
     */
    @Override
    public void on_init() {
        System.out.println("INIT PROCESS STARTING");

        robot.bddrivetrain.initAutonomous();

        robot.bdlatch.initAutonomous();

        System.out.println("Ready To Start!");

    }
    
    /**
     * Runs the opmode
     */
    @Override
    public void run() {

        System.out.println("ROBOT RUN SEQUENCE INITIALIZED!");

        runtime.reset();
        telemetry.addData("Runtime Reset", "Complete");
        System.out.println("RUNTIME RESET COMPLETE");
        robot.bddrivetrain.driveFor(FIRSTdriveinches,0.5,10);
        sleep(250);

        robot.bddrivetrain.strafeFor(SECONDstrafeinches,0.2,false, 10);
        sleep(250);

        robot.bddrivetrain.strafeFor(THIRDstrafeinches,0.2,true, 10);
        sleep(250);

        robot.bddrivetrain.driveFor(FOURTHdriveinches,0.5,10);
        sleep(250);

        robot.bddrivetrain.strafeFor(FIFTHstrafeinches,0.2, false, 10);
        sleep(250);

        robot.bddrivetrain.driveFor(SIXTHdriveinches,0.5,10);
        sleep(250);

        robot.bdlatch.closeLatch();
        sleep(250);

        robot.bddrivetrain.driveFor(SEVENTHdriveinches,0.5,10);
        sleep(250);

        robot.bdlatch.openLatch();
        sleep(250);

        robot.bddrivetrain.strafeFor(NINTHstrafeinches,0.5,true, 10);


    }


    /**
     * Runs code when the opmode stops.
     * Currently unused.
     */
    @Override
    public void on_stop() {
        //do something when the robot stops?
    }


}

