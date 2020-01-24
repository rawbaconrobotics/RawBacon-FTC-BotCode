package org.firstinspires.ftc.teamcode.BigDipper.LukeMomentAutos_OLD;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BaseLinearOpMode;
import org.firstinspires.ftc.teamcode.Uhaul.AllianceConfig;
import org.firstinspires.ftc.teamcode.Uhaul.AutonomousSelector;
import org.firstinspires.ftc.teamcode.Uhaul.OptionsConfig;

/**
 * Autonomous code to make the robot score the foundation and
 * drives under the bridge when the robot is on the blue alliance side.
 * @author Raw Bacon Coders
 */
@Autonomous(name = "Test Selector", group = "Big Dipper")

public class LukeMomentAutoBlueMiddle extends BaseLinearOpMode {
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
     * Initializes the components that the robot uses
     */
    @Override
    public void on_init() {
        System.out.println("INIT PROCESS STARTING");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        AutonomousSelector.deserializeOptions();
        telemetry.addData("TASK:", OptionsConfig.tasks.option());
        telemetry.addData("PARK:", OptionsConfig.park_middle_or_wall.option());
        AutonomousSelector.deserializeAlliance();
        if(AllianceConfig.redAlliance.redAlliancePressed() == true){
            telemetry.addData("ALLIANCE IS RED", "NICE");
        }
        else if(AllianceConfig.redAlliance.redAlliancePressed() == false){
            telemetry.addData("ALLIANCE IS BLUE", "NICE");
        }
        else{
            telemetry.addData("ALLIANCE IS BROKEN", "OOF");

        }
        telemetry.update();
        //robot.bddrivetrain.initAutonomous();

        //robot.bdlatch.initAutonomous();

        System.out.println("Ready To Start!");

    }

    /**
     * Runs the opmode
     */
    @Override
    public void run() {


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

