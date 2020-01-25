package org.firstinspires.ftc.teamcode.BigDipper.BDAutonomi.RedAlliance;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous(name= "BDAuto_ParkingOnly_StartDepot_MiddlePark_Red", group="Tank")

public class BDAuto_ParkingOnly_StartDebot_MiddlePark_Red extends BaseLinearOpMode {
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

        /*Sebastian, here is where you code the auto. If you need to code for a scenario where we don't start by
        looking at stones, just reuse one of the old lukemoment autos (add @Config at beginning for dashboard). I re-coded
        turnfor using gyros that work so I *hope*
        that it works. I'm pretty sure this is obvious but I just wanted to mention, add the tasks that the robot can do
        are under our robot class, so you will be calling all functions under robot.component.function.
        For easier testing if you make a distance into a static variable (outside of the function) then just reference it
        here, you can change the values live with FTC dashboard. Just make sure you're connected to the phone wifi
        and go to http://192.168.49.1:8080/dash and you're good. Just make sure to scroll up and click save every time you
        change something and of course because the values don't save to the code file, when you get them down, write
        them down somewhere else to transfer here!

        Oh last thing dont forget to push at the end of the day thanks

        -Luke
         */
    @Override
    public void run() {


        runtime.reset();

        robot.bddrivetrain.driveFor(24.5,.7,10);
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
