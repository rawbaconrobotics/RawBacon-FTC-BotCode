package org.firstinspires.ftc.teamcode.BigDipper;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.config.variable.BasicVariable;
import com.acmerobotics.dashboard.config.variable.CustomVariable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BaseLinearOpMode;
import org.firstinspires.ftc.teamcode.Uhaul.AutoConfig;
import org.firstinspires.ftc.teamcode.Uhaul.AutonomousSelector;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import static org.firstinspires.ftc.teamcode.Uhaul.AutonomousSelector.allianceFileName;

//DONE

/**
 * @author Raw Bacon Coders
 * Autonomous for robot
 */
//nerverest ticks
//60 1680
//40 1120
//20 560
@Config
@Autonomous(name = "2. OFFICIAL Autonomous", group = "1aa1a1a")

public class BDAutonomous extends BaseLinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();



    //0 means skystone, 1 (or 255) means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;


    private static float rectHeight = .6f / 8f;
    private static float rectWidth = 1.5f / 8f;

    private static float offsetX = 0f / 8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f / 8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive


    private static float[] midPos = {4f / 8f + offsetX, 4f / 8f + offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f / 8f + offsetX, 4f / 8f + offsetY};
    private static float[] rightPos = {6f / 8f + offsetX, 4f / 8f + offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;




    OpenCvCamera webcam;

    AutoConfig autoconfig = new AutoConfig();

    private static final String MIDDLEPARK_FOUNDATION_BLUE = "MIDDLEPARK_FOUNDATION_BLUE";
    private static final String MIDDLEPARK_STONES_BLUE = "MIDDLEPARK_STONES_BLUE";
    private static final String PARKINGONLY_STARTBZ_MIDDLEPARK_BLUE = "PARKINGONLY_STARTBZ_MIDDLEPARK_BLUE";
    private static final String PARKINGONLY_STARTBZ_WALLPARK_BLUE = "PARKINGONLY_STARTBZ_WALLPARK_BLUE";
    private static final String PARKINGONLY_STARTDEPOT_MIDDLEPARK_BLUE = "PARKINGONLY_STARTDEPOT_MIDDLEPARK_BLUE";
    private static final String PARKINGONLY_STARTDEPOT_WALLPARK_BLUE = "PARKINGONLY_STARTDEPOT_MIDDLEPARK_BLUE";
    private static final String WALLPARK_FOUNDATION_BLUE = "WALLPARK_FOUNDATION_BLUE";
    private static final String WALLPARK_STONES_BLUE = "WALLPARK_STONES_BLUE";
    private static final String MIDDLEPARK_STONESANDFOUNDATION_BLUE = "MIDDLEPARK_STONESANDFOUNDATION_BLUE";
    private static final String WALLPARK_STONESANDFOUNDATION_BLUE = "WALLPARK_STONESANDFOUNDATION_BLUE";

    private static final String MIDDLEPARK_FOUNDATION_RED = "MIDDLEPARK_FOUNDATION_RED";
    private static final String MIDDLEPARK_STONES_RED = "MIDDLEPARK_STONES_RED";
    private static final String PARKINGONLY_STARTBZ_MIDDLEPARK_RED = "PARKINGONLY_STARTBZ_MIDDLEPARK_RED";
    private static final String PARKINGONLY_STARTBZ_WALLPARK_RED = "PARKINGONLY_STARTBZ_WALLPARK_RED";
    private static final String PARKINGONLY_STARTDEPOT_MIDDLEPARK_RED = "PARKINGONLY_STARTDEPOT_MIDDLEPARK_RED";
    private static final String PARKINGONLY_STARTDEPOT_WALLPARK_RED = "PARKINGONLY_STARTDEPOT_MIDDLEPARK_RED";
    private static final String WALLPARK_FOUNDATION_RED = "WALLPARK_FOUNDATION_RED";
    private static final String WALLPARK_STONES_RED = "WALLPARK_STONES_RED";
    private static final String MIDDLEPARK_STONESANDFOUNDATION_RED = "MIDDLEPARK_STONESANDFOUNDATION_RED";
    private static final String WALLPARK_STONESANDFOUNDATION_RED = "WALLPARK_STONESANDFOUNDATION_RED";


    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private String catName;
    private CustomVariable catVar;


    /**
     * Defines a proccess that starts when initialization occurs
     */
    @Override
    public void on_init() {


        robot.bddrivetrain.initAutonomous();
        robot.bdlatch.initAutonomous();
        robot.bdtapemeasure.initAutonomous();
        robot.bdgrabber.initAutonomous();
        robot.bdcapstone.initAutonomous();


        switch (autoconfig.tasks) {
            case DO_FOUNDATION:
                telemetry.addData("Task", " FOUNDATION");

                break;
            case DO_STONE:
                telemetry.addData("Task", " STONE");

                break;
            case DO_BOTH:
                telemetry.addData("Task", " BOTH");

                break;
            case STARTING_DEPOT_SIDE:
                telemetry.addData("Task", " PARK ONLY START DEPOT");

                break;
            case STARTING_BUILDER_SIDE:
                telemetry.addData("Task", " PARK ONLY START BUILD");

                break;
        }

        switch (autoconfig.redAlliance) {
            case RED:
                telemetry.addData("Alliance", " Red");

                break;
            case BLUE:
                telemetry.addData("Alliance", " Blue");

                break;
        }

        switch (autoconfig.park) {
            case PARK_MIDDLE:
                telemetry.addData("Park", " MIDDLE");

                break;
            case PARK_WALL:
                telemetry.addData("Park", " WALL");

                break;
        }

        telemetry.update();


        //The following commented out code is what we would use if we didn't have a webcam.
        //  int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //  phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //  phoneCam.openCameraDevice();//open camera
        //  phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        //  phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.


        if ((autoconfig.tasks == AutonomousSelector.Options.DO_STONE) || (autoconfig.tasks == AutonomousSelector.Options.DO_BOTH)) {

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            webcam.openCameraDevice();//open camera
            webcam.setPipeline(new BDAutonomous.StageSwitchingPipeline());//different stages
            webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
            //width, height
            //width = height in this case, because camera is in portrait mode.

            while (!opModeIsActive() && !isStopRequested()) {
                telemetry.addData("Values", valLeft + "   " + valMid + "   " + valRight);
                telemetry.addData("Height", rows);
                telemetry.addData("Width", cols);

                if (isStopRequested()) {
                    webcam.stopStreaming();
                }
            }
        }
    }

    /**
     * Runs the process
     */



    @Override
    public void run() {
        if ((autoconfig.tasks == AutonomousSelector.Options.DO_STONE) || (autoconfig.tasks == AutonomousSelector.Options.DO_BOTH)) {
            webcam.stopStreaming();
        }

        if (autoconfig.redAlliance == AutonomousSelector.Alliance.RED) {
            switch (autoconfig.tasks) {
                case DO_FOUNDATION:
                    switch (autoconfig.park) {
                        case PARK_MIDDLE:
                            runtime.reset();

                            robot.bddrivetrain.driveFor(-47, -.8, 5);
                            sleep(100);
                            robot.bddrivetrain.strafeFor(-10,.7,4);
                            sleep(100);
                            robot.bddrivetrain.driveFor(19, .8,4);
                            sleep(100);
                            robot.bddrivetrain.strafeFor(-28,.8,7);
                            sleep(100);
                            robot.bddrivetrain.driveFor(-7,-.7,7);
                            sleep(100);
                            robot.bdlatch.closeLatch();
                            sleep(1000);
                            robot.bddrivetrain.driveFor(38,.8,7);
                            sleep(100);
                            robot.bddrivetrain.driveFor(-1,-.8,7);
                            sleep(100);
                            robot.bdlatch.openLatch();
                            sleep(1000);
                            robot.bddrivetrain.strafeFor(40,.8,5);
                            sleep(100);
                            robot.bddrivetrain.driveFor(-25,-.8,5);
                            sleep(100);
                            robot.bddrivetrain.strafeFor(20,.8,4);
                            sleep(100);


                            telemetry.addData("PATH", "COMPLETE");
                            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);                            break;
                        case PARK_WALL:
                            runtime.reset();

                            robot.bddrivetrain.driveFor(-48, -.8, 5);
                            sleep(100);
                            robot.bddrivetrain.strafeFor(-10,.7,4);
                            sleep(100);
                            robot.bddrivetrain.driveFor(19, .8,4);
                            sleep(100);
                            robot.bddrivetrain.strafeFor(-28,.8,7);
                            sleep(100);
                            robot.bddrivetrain.driveFor(-7,-.7,7);
                            sleep(100);
                            robot.bdlatch.closeLatch();
                            sleep(1000);
                            robot.bddrivetrain.driveFor(38,.8,7);
                            sleep(100);
                            robot.bddrivetrain.driveFor(-1,-.8,7);
                            sleep(100);
                            robot.bdlatch.openLatch();
                            sleep(1000);
                            robot.bddrivetrain.strafeFor(((40)+(20)),.8,5);
                            sleep(100);



                            telemetry.addData("PATH", "COMPLETE");
                            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);                            break;
                    }
                    break;
                case DO_STONE:
                    switch (autoconfig.park) {
                        case PARK_MIDDLE:

                            webcam.stopStreaming();

                            runtime.reset();

                            robot.bddrivetrain.driveFor(28,1,10);
                            sleep(500);

                            if (valLeft == 0) { // stone is on left, run left path

                                robot.bddrivetrain.strafeFor(-8, 1, 10);
                                sleep(500);
                                robot.bddrivetrain.driveFor(15,1,10);
                                sleep(500);
                                robot.bdgrabber.grabDownAuto();
                                sleep(500);
                                robot.bddrivetrain.driveFor(-19,-1,10);
                                sleep(500);
                                robot.bddrivetrain.strafeFor(8, 1, 10);
                                sleep(500);


                            } else if (valMid == 0) { // stone is in middle, run middle path

                                robot.bddrivetrain.driveFor(15,1,10);
                                sleep(500);
                                robot.bdgrabber.grabDownAuto();
                                sleep(500);
                                robot.bddrivetrain.driveFor(-19,-1,10);
                                sleep(500);


                            } else if (valRight == 0) { //stone on right, run right path

                                robot.bddrivetrain.strafeFor(7, 1, 10);
                                sleep(500);
                                robot.bddrivetrain.driveFor(15,1,10);
                                sleep(500);
                                robot.bdgrabber.grabDownAuto();
                                sleep(500);
                                robot.bddrivetrain.driveFor(-19,-1,10);
                                sleep(500);
                                robot.bddrivetrain.strafeFor(-8, 1, 10);
                                sleep(500);


                            } else {
                                robot.bddrivetrain.driveFor(15,1,10);
                                sleep(500);
                                robot.bdgrabber.grabDownAuto();
                                sleep(500);
                                robot.bddrivetrain.driveFor(-19,-1,10);
                                //skystone location cannot be determined, either try for a random one or just grab the foundation

                            }

                            sleep(500);
                            robot.bddrivetrain.turnFor(85, 1, 15);


                            sleep(500);
                            robot.bddrivetrain.driveFor(54,1,10);
                            sleep(500);
                            robot.bdgrabber.grabUpAuto();
                            sleep(500);
                            robot.bddrivetrain.driveFor(-22,-1,10);


                            telemetry.addData("PATH", "COMPLETE");
                            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);                            break;
                        case PARK_WALL:

                            webcam.stopStreaming();

                            runtime.reset();

                            robot.bddrivetrain.driveFor(28,1,10);
                            sleep(500);

                            if (valLeft == 0) { // stone is on left, run left path

                                robot.bddrivetrain.strafeFor(8, 1, 10);
                                sleep(500);
                                robot.bddrivetrain.driveFor(15,1,10);
                                sleep(500);
                                robot.bdgrabber.grabDownAuto();
                                sleep(500);
                                robot.bddrivetrain.driveFor(-19,-1,10);
                                sleep(500);
                                robot.bddrivetrain.strafeFor(-8, 1, 10);
                                sleep(500);


                            } else if (valMid == 0) { // stone is in middle, run middle path

                                robot.bddrivetrain.driveFor(15,1,10);
                                sleep(500);
                                robot.bdgrabber.grabDownAuto();
                                sleep(500);
                                robot.bddrivetrain.driveFor(-19,-1,10);
                                sleep(500);


                            } else if (valRight == 0) { //stone on right, run right path

                                robot.bddrivetrain.strafeFor(-8, 1, 10);
                                sleep(500);
                                robot.bddrivetrain.driveFor(15,1,10);
                                sleep(500);
                                robot.bdgrabber.grabDownAuto();
                                sleep(500);
                                robot.bddrivetrain.driveFor(-19,-1,10);
                                sleep(500);
                                robot.bddrivetrain.strafeFor(7, 1, 10);
                                sleep(500);


                            } else {
                                robot.bddrivetrain.driveFor(15,1,10);
                                sleep(500);
                                robot.bdgrabber.grabDownAuto();
                                sleep(500);
                                robot.bddrivetrain.driveFor(-19,-1,10);
                                //skystone location cannot be determined, either try for a random one or just grab the foundation

                            }

                            sleep(500);
                            robot.bddrivetrain.driveFor(-27.5,-1,10);
                            robot.bddrivetrain.turnFor(85, 1, 15);


                            sleep(500);
                            robot.bddrivetrain.driveFor(54,1,10);
                            sleep(500);
                            robot.bdgrabber.grabUpAuto();
                            sleep(500);
                            robot.bddrivetrain.driveFor(-22,-1,10);


                            telemetry.addData("PATH", "COMPLETE");
                            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);
                            break;
                    }
                    break;
                case DO_BOTH:
                    switch (autoconfig.park) {
                        case PARK_MIDDLE:

                            break;
                        case PARK_WALL:

                            break;
                    }
                    break;
                case STARTING_DEPOT_SIDE:
                    switch (autoconfig.park) {
                        case PARK_MIDDLE:
                            runtime.reset();

                            robot.bddrivetrain.driveFor(24.5,.7,10);
                            robot.bddrivetrain.strafeFor(12.5, .7, 15);
                            sleep(500);
                            break;
                        case PARK_WALL:
                            runtime.reset();

                            robot.bddrivetrain.strafeFor(12.5, .7, 15);
                            sleep(500);
                            break;
                    }
                    break;
                case STARTING_BUILDER_SIDE:
                    switch (autoconfig.park) {
                        case PARK_MIDDLE:
                            runtime.reset();

                            robot.bddrivetrain.driveFor(24.5,.7,10);
                            robot.bddrivetrain.strafeFor(-12.5, .7, 15);
                            sleep(500);                            break;
                        case PARK_WALL:
                            runtime.reset();

                            robot.bddrivetrain.strafeFor(-12.5, .7, 15);
                            sleep(500);                            break;
                    }
                    break;

            }

        } else {
            switch (autoconfig.tasks) {
                case DO_FOUNDATION:
                    switch (autoconfig.park) {
                        case PARK_MIDDLE:

                            runtime.reset();


                            robot.bddrivetrain.driveFor(-48, -.8, 5);
                            sleep(100);
                            robot.bddrivetrain.strafeFor(10,.7,4);
                            sleep(100);
                            robot.bddrivetrain.driveFor(16, .8,4);
                            sleep(100);
                            robot.bddrivetrain.strafeFor(34,.8,7);
                            sleep(100);
                            robot.bddrivetrain.driveFor(-7,-.7,7);
                            sleep(100);
                            robot.bdlatch.closeLatch();
                            sleep(1000);
                            robot.bddrivetrain.driveFor(38,.8,7);
                            sleep(100);
                            robot.bddrivetrain.driveFor(-1,-.8,7);
                            sleep(100);
                            robot.bdlatch.openLatch();
                            sleep(1000);
                            robot.bddrivetrain.strafeFor(-40,.8,5);
                            sleep(100);
                            robot.bddrivetrain.driveFor(-23,-.8,5);
                            sleep(100);
                            robot.bddrivetrain.strafeFor(-20,.8,4);
                            sleep(100);


                            telemetry.addData("PATH", "COMPLETE");
                            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);
                            break;
                        case PARK_WALL:

                            runtime.reset();


                            robot.bddrivetrain.driveFor(-48, -.8, 5);
                            sleep(100);
                            robot.bddrivetrain.strafeFor(10,.7,4);
                            sleep(100);
                            robot.bddrivetrain.driveFor(16, .8,4);
                            sleep(100);
                            robot.bddrivetrain.strafeFor(34,.8,7);
                            sleep(100);
                            robot.bddrivetrain.driveFor(-7,-.7,7);
                            sleep(100);
                            robot.bdlatch.closeLatch();
                            sleep(1000);
                            robot.bddrivetrain.driveFor(38,.8,7);
                            sleep(100);
                            robot.bddrivetrain.driveFor(-1,-.8,7);
                            sleep(100);
                            robot.bdlatch.openLatch();
                            sleep(1000);
                            robot.bddrivetrain.strafeFor(((-40)+(-20)),.8,5);
                            sleep(100);



                            telemetry.addData("PATH", "COMPLETE");
                            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);
                            break;
                    }
                    break;
                case DO_STONE:
                    switch (autoconfig.park) {
                        case PARK_MIDDLE:
                            webcam.stopStreaming();

                            runtime.reset();

                            robot.bddrivetrain.driveFor(28,1,10);
                            sleep(500);

                            if (valLeft == 0) { // stone is on left, run left path

                                robot.bddrivetrain.strafeFor(-8, 1, 10);
                                sleep(500);
                                robot.bddrivetrain.driveFor(15,1,10);
                                sleep(500);
                                robot.bdgrabber.grabDownAuto();
                                sleep(500);
                                robot.bddrivetrain.driveFor(-19,-1,10);
                                sleep(500);
                                robot.bddrivetrain.strafeFor(8, 1, 10);
                                sleep(500);


                            } else if (valMid == 0) { // stone is in middle, run middle path

                                robot.bddrivetrain.driveFor(15,1,10);
                                sleep(500);
                                robot.bdgrabber.grabDownAuto();
                                sleep(500);
                                robot.bddrivetrain.driveFor(-19,-1,10);
                                sleep(500);


                            } else if (valRight == 0) { //stone on right, run right path

                                robot.bddrivetrain.strafeFor(8, 1, 10);
                                sleep(500);
                                robot.bddrivetrain.driveFor(15,1,10);
                                sleep(500);
                                robot.bdgrabber.grabDownAuto();
                                sleep(500);
                                robot.bddrivetrain.driveFor(-19,-1,10);
                                sleep(500);
                                robot.bddrivetrain.strafeFor(-7, 1, 10);
                                sleep(500);


                            } else {
                                robot.bddrivetrain.driveFor(15,1,10);
                                sleep(500);
                                robot.bdgrabber.grabDownAuto();
                                sleep(500);
                                robot.bddrivetrain.driveFor(-19,-1,10);
                                //skystone location cannot be determined, either try for a random one or just grab the foundation

                            }

                            sleep(500);
                            robot.bddrivetrain.turnFor(-80, 1, 15);

            /*
            //TURNING SAW THE ACTUAL VALUES AS POTITIVE!!!!!!!
            //KEPT GOING FOREVER TURNING RIGHT
            --nevermind, seems to be fixed
             */

                            sleep(500);
                            robot.bddrivetrain.driveFor(54, 1, 10);
                            sleep(500);
                            robot.bdgrabber.grabUpAuto();
                            sleep(500);
                            robot.bddrivetrain.driveFor(-22, -1, 10);


                            telemetry.addData("PATH", "COMPLETE");
                            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);
                            ////////////////////////////////////////////////

                            break;
                        case PARK_WALL:

                            webcam.stopStreaming();

                            runtime.reset();

                            robot.bddrivetrain.driveFor(28,1,10);
                            sleep(500);

                            if (valLeft == 0) { // stone is on left, run left path

                                robot.bddrivetrain.strafeFor(-8, 1, 10);
                                sleep(500);
                                robot.bddrivetrain.driveFor(15,1,10);
                                sleep(500);
                                robot.bdgrabber.grabDownAuto();
                                sleep(500);
                                robot.bddrivetrain.driveFor(-19,-1,10);
                                sleep(500);
                                robot.bddrivetrain.strafeFor(8, 1, 10);
                                sleep(500);


                            } else if (valMid == 0) { // stone is in middle, run middle path

                                robot.bddrivetrain.driveFor(15,1,10);
                                sleep(500);
                                robot.bdgrabber.grabDownAuto();
                                sleep(500);
                                robot.bddrivetrain.driveFor(-19,-1,10);
                                sleep(500);


                            } else if (valRight == 0) { //stone on right, run right path

                                robot.bddrivetrain.strafeFor(8, 1, 10);
                                sleep(500);
                                robot.bddrivetrain.driveFor(15,1,10);
                                sleep(500);
                                robot.bdgrabber.grabDownAuto();
                                sleep(500);
                                robot.bddrivetrain.driveFor(-19,-1,10);
                                sleep(500);
                                robot.bddrivetrain.strafeFor(-7, 1, 10);
                                sleep(500);


                            } else {
                                robot.bddrivetrain.driveFor(15,1,10);
                                sleep(500);
                                robot.bdgrabber.grabDownAuto();
                                sleep(500);
                                robot.bddrivetrain.driveFor(-19,-1,10);
                                //skystone location cannot be determined, either try for a random one or just grab the foundation

                            }

                            sleep(500);
                            robot.bddrivetrain.driveFor(-27.5,-1,10);
                            robot.bddrivetrain.turnFor(-85, 1, 15);



                            sleep(500);
                            robot.bddrivetrain.driveFor(54,1,10);
                            sleep(500);
                            robot.bdgrabber.grabUpAuto();
                            sleep(500);
                            robot.bddrivetrain.driveFor(-22,-1,10);


                            telemetry.addData("PATH", "COMPLETE");
                            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);
                            break;
                    }
                    break;
                case DO_BOTH:
                    switch (autoconfig.park) {
                        case PARK_MIDDLE:

                            break;
                        case PARK_WALL:

                            break;
                    }
                    break;
                case STARTING_DEPOT_SIDE:
                    switch (autoconfig.park) {
                        case PARK_MIDDLE:

                            runtime.reset();

                            robot.bddrivetrain.driveFor(24.5,.7,10);
                            robot.bddrivetrain.strafeFor(-12.5, .7, 15);
                            sleep(500);                            break;
                        case PARK_WALL:
                            runtime.reset();

                            robot.bddrivetrain.strafeFor(-12.5, .7, 15);
                            sleep(500);
                            break;
                    }
                    break;
                case STARTING_BUILDER_SIDE:
                    switch (autoconfig.park) {
                        case PARK_MIDDLE:

                            runtime.reset();

                            robot.bddrivetrain.driveFor(24.5,.7,10);
                            robot.bddrivetrain.strafeFor(12.5, .7, 15);
                            sleep(500);
                            break;
                        case PARK_WALL:

                            runtime.reset();

                            robot.bddrivetrain.strafeFor(12.5, .7, 15);
                            sleep(500);
                            break;
                    }
                    break;

            }

        }

    }

        //insert auto here


    /**
     * What the robot should do when it sees the stop button was pressed / timer ended
     */
    public void on_stop() {
        robot.bddrivetrain.stopDrive();
    }

    //detection pipeline --- don't mess with any of this unless CV detection area is sketchy
    static class StageSwitchingPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();


        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if (nextStageNum >= stages.length) {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int) (input.rows() * midPos[1]), (int) (input.cols() * midPos[0]));//gets value at circle
            valMid = (int) pixMid[0];

            double[] pixLeft = thresholdMat.get((int) (input.rows() * leftPos[1]), (int) (input.cols() * leftPos[0]));//gets value at circle
            valLeft = (int) pixLeft[0];

            double[] pixRight = thresholdMat.get((int) (input.rows() * rightPos[1]), (int) (input.cols() * rightPos[0]));//gets value at circle
            valRight = (int) pixRight[0];

            //create three points
            Point pointMid = new Point((int) (input.cols() * midPos[0]), (int) (input.rows() * midPos[1]));
            Point pointLeft = new Point((int) (input.cols() * leftPos[0]), (int) (input.rows() * leftPos[1]));
            Point pointRight = new Point((int) (input.cols() * rightPos[0]), (int) (input.rows() * rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid, 5, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointLeft, 5, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointRight, 5, new Scalar(255, 0, 0), 1);//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols() * (leftPos[0] - rectWidth / 2),
                            input.rows() * (leftPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (leftPos[0] + rectWidth / 2),
                            input.rows() * (leftPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols() * (midPos[0] - rectWidth / 2),
                            input.rows() * (midPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (midPos[0] + rectWidth / 2),
                            input.rows() * (midPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols() * (rightPos[0] - rectWidth / 2),
                            input.rows() * (rightPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (rightPos[0] + rectWidth / 2),
                            input.rows() * (rightPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport) {
                case THRESHOLD: {
                    return thresholdMat;
                }

                case detection: {
                    return all;
                }

                case RAW_IMAGE: {
                    return input;
                }

                default: {
                    return input;
                }
            }
        }

    }




}
