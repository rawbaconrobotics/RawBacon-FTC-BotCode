package org.firstinspires.ftc.teamcode.BigDipper;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.BigDipper.BDAutonomi.BlueAlliance.BDAuto_MiddlePark_Foundation_Blue;
import org.firstinspires.ftc.teamcode.BigDipper.BDAutonomi.BlueAlliance.BDAuto_MiddlePark_Foundation_Red;
import org.firstinspires.ftc.teamcode.BigDipper.BDAutonomi.BlueAlliance.BDAuto_MiddlePark_Stones_Blue;
import org.firstinspires.ftc.teamcode.BigDipper.BDAutonomi.BlueAlliance.BDAuto_ParkingOnly_StartBZ_MiddlePark_Blue;
import org.firstinspires.ftc.teamcode.BigDipper.BDAutonomi.BlueAlliance.BDAuto_ParkingOnly_StartBZ_WallPark_Blue;
import org.firstinspires.ftc.teamcode.BigDipper.BDAutonomi.BlueAlliance.BDAuto_ParkingOnly_StartDebot_MiddlePark_Blue;
import org.firstinspires.ftc.teamcode.BigDipper.BDAutonomi.BlueAlliance.BDAuto_ParkingOnly_StartDepot_WallPark_Blue;
import org.firstinspires.ftc.teamcode.BigDipper.BDAutonomi.BlueAlliance.BDAuto_WallPark_Foundation_Blue;
import org.firstinspires.ftc.teamcode.BigDipper.BDAutonomi.BlueAlliance.BDAuto_WallPark_Stones_Blue;
import org.firstinspires.ftc.teamcode.BigDipper.BDAutonomi.RedAlliance.BDAuto_MiddlePark_Stones_Red;
import org.firstinspires.ftc.teamcode.BigDipper.BDAutonomi.RedAlliance.BDAuto_ParkingOnly_StartBZ_MiddlePark_Red;
import org.firstinspires.ftc.teamcode.BigDipper.BDAutonomi.RedAlliance.BDAuto_ParkingOnly_StartBZ_WallPark_Red;
import org.firstinspires.ftc.teamcode.BigDipper.BDAutonomi.RedAlliance.BDAuto_ParkingOnly_StartDebot_MiddlePark_Red;
import org.firstinspires.ftc.teamcode.BigDipper.BDAutonomi.RedAlliance.BDAuto_ParkingOnly_StartDepot_WallPark_Red;
import org.firstinspires.ftc.teamcode.BigDipper.BDAutonomi.RedAlliance.BDAuto_WallPark_Foundation_Red;
import org.firstinspires.ftc.teamcode.BigDipper.BDAutonomi.RedAlliance.BDAuto_WallPark_Stones_Red;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BaseLinearOpMode;
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
import org.firstinspires.ftc.teamcode.Uhaul.AllianceConfig;
import org.firstinspires.ftc.teamcode.Uhaul.AutonomousSelector;
import org.firstinspires.ftc.teamcode.Uhaul.OptionsConfig;

import java.util.ArrayList;
import java.util.List;

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
@Autonomous(name = "OFFICIAL Autonomous", group = "1aa1a1a")

public class BDAutonomous extends BaseLinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public static double one = 48;
    public static double two = 10;
    public static double three = 16;
    public static double four = 28;
    public static double fourpointfive = 7;
    public static double five = 38;
    public static double fivepointfive = 1;
    public static double six = 34;
    public static double seven = 27;
    public static double eight = 20;
    static double nine = 0;
    static double ten = 0;
    static double eleven = 0;


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

        AutonomousSelector.deserializeOptions();


        telemetry.addData("PARK:", OptionsConfig.park.option());
        AutonomousSelector.deserializeAlliance();


        //The following commented out code is what we would use if we didn't have a webcam.
        //  int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //  phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //  phoneCam.openCameraDevice();//open camera
        //  phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        //  phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.


        if (OptionsConfig.tasks.option().equals("DO STONE") || OptionsConfig.tasks.option().equals("DO BOTH")) {

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

                telemetry.update();
            }
            if (isStopRequested()) {
                webcam.stopStreaming();
            }
        }
    }

    /**
     * Runs the process
     */



    @Override
    public void run()  {
        BDAuto_MiddlePark_Foundation_Red middleParkFoundationRed = new BDAuto_MiddlePark_Foundation_Red();
        BDAuto_MiddlePark_Stones_Red middleParkStonesRed = new BDAuto_MiddlePark_Stones_Red();
        BDAuto_ParkingOnly_StartBZ_MiddlePark_Red parkingOnlyStartBZMiddleParkRed = new BDAuto_ParkingOnly_StartBZ_MiddlePark_Red();
        BDAuto_ParkingOnly_StartBZ_WallPark_Red parkingOnlyStartBZWallParkRed = new BDAuto_ParkingOnly_StartBZ_WallPark_Red();
        BDAuto_ParkingOnly_StartDebot_MiddlePark_Red parkingOnlyStartDebotMiddleParkRed = new BDAuto_ParkingOnly_StartDebot_MiddlePark_Red();
        BDAuto_ParkingOnly_StartDepot_WallPark_Red parkingOnlyStartDepotWallParkRed = new BDAuto_ParkingOnly_StartDepot_WallPark_Red();
        BDAuto_WallPark_Foundation_Red wallParkFoundationRed = new BDAuto_WallPark_Foundation_Red();
        BDAuto_WallPark_Stones_Red wallParkStonesRed = new BDAuto_WallPark_Stones_Red();
        BDAuto_MiddlePark_Foundation_Blue middleParkFoundationBlue = new BDAuto_MiddlePark_Foundation_Blue();
        BDAuto_MiddlePark_Stones_Blue middleParkStonesBlue = new BDAuto_MiddlePark_Stones_Blue();
        BDAuto_ParkingOnly_StartBZ_MiddlePark_Blue parkingOnlyStartBZMiddleParkBlue = new BDAuto_ParkingOnly_StartBZ_MiddlePark_Blue();
        BDAuto_ParkingOnly_StartBZ_WallPark_Blue parkingOnlyStartBZWallParkBlue = new BDAuto_ParkingOnly_StartBZ_WallPark_Blue();
        BDAuto_ParkingOnly_StartDebot_MiddlePark_Blue parkingOnlyStartDebotMiddleParkBlue = new BDAuto_ParkingOnly_StartDebot_MiddlePark_Blue();
        BDAuto_ParkingOnly_StartDepot_WallPark_Blue parkingOnlyStartDepotWallParkBlue = new BDAuto_ParkingOnly_StartDepot_WallPark_Blue();
        BDAuto_WallPark_Foundation_Blue wallParkFoundationBlue = new BDAuto_WallPark_Foundation_Blue();
        BDAuto_WallPark_Stones_Blue wallParkStonesBlue = new BDAuto_WallPark_Stones_Blue();


        if (AllianceConfig.redAlliance.redAlliancePressed()) {
            switch (OptionsConfig.tasks) {
                case DO_FOUNDATION:
                    switch (OptionsConfig.park) {
                        case PARK_MIDDLE:
                            middleParkFoundationRed.run();
                            break;
                        case PARK_WALL:
                            wallParkFoundationRed.run();
                            break;
                    }
                    break;
                case DO_STONE:
                    switch (OptionsConfig.park) {
                        case PARK_MIDDLE:
                            middleParkStonesRed.run();
                            break;
                        case PARK_WALL:
                            wallParkStonesRed.run();
                            break;
                    }
                    break;
                case DO_BOTH:
                    switch (OptionsConfig.park) {
                        case PARK_MIDDLE:

                            break;
                        case PARK_WALL:

                            break;
                    }
                    break;
                case STARTING_DEPOT_SIDE:
                    switch (OptionsConfig.park) {
                        case PARK_MIDDLE:
                            parkingOnlyStartDebotMiddleParkRed.run();
                            break;
                        case PARK_WALL:
                            parkingOnlyStartDepotWallParkRed.run();
                            break;
                    }
                    break;
                case STARTING_BUILDER_SIDE:
                    switch (OptionsConfig.park) {
                        case PARK_MIDDLE:
                            parkingOnlyStartBZMiddleParkRed.run();
                            break;
                        case PARK_WALL:
                            parkingOnlyStartBZWallParkRed.run();
                            break;
                    }
                    break;

            }

        } else {
            switch (OptionsConfig.tasks) {
                case DO_FOUNDATION:
                    switch (OptionsConfig.park) {
                        case PARK_MIDDLE:
                            middleParkFoundationBlue.run();
                            break;
                        case PARK_WALL:
                            wallParkFoundationBlue.run();
                            break;
                    }
                    break;
                case DO_STONE:
                    switch (OptionsConfig.park) {
                        case PARK_MIDDLE:
                            middleParkStonesBlue.run();
                            break;
                        case PARK_WALL:
                            wallParkStonesBlue.run();
                            break;
                    }
                    break;
                case DO_BOTH:
                    switch (OptionsConfig.park) {
                        case PARK_MIDDLE:

                            break;
                        case PARK_WALL:

                            break;
                    }
                    break;
                case STARTING_DEPOT_SIDE:
                    switch (OptionsConfig.park) {
                        case PARK_MIDDLE:
                            parkingOnlyStartDebotMiddleParkBlue.run();
                            break;
                        case PARK_WALL:
                            parkingOnlyStartDepotWallParkBlue.run();
                            break;
                    }
                    break;
                case STARTING_BUILDER_SIDE:
                    switch (OptionsConfig.park) {
                        case PARK_MIDDLE:
                            parkingOnlyStartBZMiddleParkBlue.run();
                            break;
                        case PARK_WALL:
                            parkingOnlyStartBZWallParkBlue.run();
                            break;
                    }
                    break;

            }

        }

    }


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
