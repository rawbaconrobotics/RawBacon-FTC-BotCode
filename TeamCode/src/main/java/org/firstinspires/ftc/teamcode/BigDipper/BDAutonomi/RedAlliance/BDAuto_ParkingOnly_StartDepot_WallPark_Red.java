package org.firstinspires.ftc.teamcode.BigDipper.BDAutonomi.RedAlliance;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BaseLinearOpMode;


/**
 * @author Raw Bacon Coders
 * Autonomous for robot
 */
 //nerverest ticks
 //60 1680
 //40 1120
 //20 560
@Config
@Disabled
@Autonomous(name= "BDAuto_ParkingOnly_StartDepot_WallPark_Red", group="Tank")

public class BDAuto_ParkingOnly_StartDepot_WallPark_Red extends BaseLinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 (or 255) means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255


    /**
     * Defines a proccess that starts when initialization occurs
     */
    @Override
    public void on_init() {
        System.out.println("INIT PROCESS STARTING");
        robot.bddrivetrain.initAutonomous();
        robot.bdlatch.initAutonomous();
        robot.bdtapemeasure.initAutonomous();
        robot.bdgrabber.initAutonomous();
        robot.bdcapstone.initAutonomous();

        //The following commented out code is what we would use if we didn't have a webcam.
        //  int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //  phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //  phoneCam.openCameraDevice();//open camera
        //  phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        //  phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

    }

    /**
     * Runs the process
     */


    @Override
    public void run() {


        runtime.reset();

        robot.bddrivetrain.strafeFor(12.5, .7, 15);
        sleep(500);


    }

    //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);




        /** What the robot should do when it sees the stop button was pressed / timer ended */
        public void on_stop() {
            robot.bddrivetrain.stopDrive();
        }
    //detection pipeline --- don't mess with any of this unless CV detection area is sketchy





}
