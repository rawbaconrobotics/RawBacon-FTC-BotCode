package org.firstinspires.ftc.teamcode.Uhaul.UhaulAuto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BDLatch;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BaseLinearOpMode;
import org.firstinspires.ftc.teamcode.Uhaul.UhaulLinearOpMode;

@Config
@Disabled
@Autonomous(name = "UhaulRedFoundWall", group = "Uhaul")

public class UhaulRedFoundWall extends UhaulLinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public static double FIRSTdriveinches = -39;
    public static double SECONDstrafeinches = 7;
    public static double THIRDstrafeinches = 4;
    public static double FOURTHdriveinches = 25;
    public static double FIFTHstrafeinches = 28;
    public static double SIXTHdriveinches = -15;
    public static double SEVENTHdriveinches = 28;
    public static double NINTHstrafeinches = 48;

    @Override
    public void on_init() {
        System.out.println("INIT PROCESS STARTING");

        robot.drive.initAutonomous();
        robot.latch.initAutonomous();

        System.out.println("Ready To Start!");
    }
    @Override
    public void run() {

        System.out.println("ROBOT RUN SEQUENCE INITIALIZED!");

        runtime.reset();
        telemetry.addData("Runtime Reset", "Complete");
        System.out.println("RUNTIME RESET COMPLETE");
        robot.drive.driveFor(FIRSTdriveinches,0.5,10);
        sleep(250);

        robot.drive.strafeFor(SECONDstrafeinches,0.2,true, 10);
        sleep(250);

        robot.drive.strafeFor(THIRDstrafeinches,0.2,false, 10);
        sleep(250);

        robot.drive.driveFor(FOURTHdriveinches,0.5,10);
        sleep(250);

        robot.drive.strafeFor(FIFTHstrafeinches,0.2, true, 10);
        sleep(250);

        robot.drive.driveFor(SIXTHdriveinches,0.5,10);
        sleep(250);

        robot.latch.closeLatch();
        sleep(250);

        robot.drive.driveFor(SEVENTHdriveinches,0.5,10);
        sleep(250);

        robot.latch.openLatch();
        sleep(250);

        robot.drive.strafeFor(NINTHstrafeinches,0.5,false, 10);


    }
    @Override
    public void on_stop() {
        //do something when the robot stops?
    }
}

