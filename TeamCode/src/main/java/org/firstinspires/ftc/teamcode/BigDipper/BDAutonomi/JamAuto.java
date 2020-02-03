package org.firstinspires.ftc.teamcode.BigDipper.BDAutonomi;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BaseLinearOpMode;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Raw Bacon Coders
 * Autonomous for robot
 */
@Config
@Autonomous(name = "JAM Autonomous", group = "2bb2b2b22b")
@Disabled
public class JamAuto extends BaseLinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

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

    private JamSelector.JAlliance alliance;
    private JamSelector.JGoal goal;
    private JamSelector.JDestination destination;


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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();//open camera
        webcam.setPipeline(new StageSwitchingPipeline());//different stages
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

        alliance = JamConfig.alliance;
        destination = JamConfig.destination;
        goal = JamConfig.goal;
        telemetry.addData("Alliance: ", alliance);
        telemetry.addData("Goals: ", goal);
        telemetry.addData("Destiantion: ", destination);
        telemetry.update();
    }

    @Override
    public void run() {
        switch (goal) {
            case FOUNDATION:
                foundation();
                break;
            case PARK:
                park();
                break;
            case STONES:
                //TODO make stones method
                break;
        }
        telemetry.addData("PATH", "COMPLETE");
    }

    public void foundation() {
        int s = 0;
        switch (alliance) {
            case RED:
                s = -1;
                break;
            case BLUE:
                s = 1;
                break;
        }
        robot.bddrivetrain.driveFor(-48, -.8, 5);
        sleep(100);
        robot.bddrivetrain.strafeFor(10 * s, .7, 4);
        sleep(100);
        robot.bddrivetrain.driveFor(16, .8, 4);
        sleep(100);
        robot.bddrivetrain.strafeFor(34 * s, .8, 7);
        sleep(100);
        robot.bddrivetrain.driveFor(-7, -.7, 7);
        sleep(100);
        robot.bdlatch.closeLatch();
        sleep(1000);
        robot.bddrivetrain.driveFor(38, .8, 7);
        sleep(100);
        robot.bddrivetrain.driveFor(-1, -.8, 7);
        sleep(100);
        robot.bdlatch.openLatch();
        sleep(1000);
        robot.bddrivetrain.strafeFor(-40 * s, .8, 5);
        sleep(100);
        if (destination == JamSelector.JDestination.MIDDLE) {
            robot.bddrivetrain.driveFor(-23, -.8, 5);
            sleep(100);
        }
        robot.bddrivetrain.strafeFor(-20 * s, .8, 4);
    }

    public void park() {
        if (destination == JamSelector.JDestination.MIDDLE) {
            int s = 0;
            switch (alliance) {
                case RED:
                    s = -1;
                    break;
                case BLUE:
                    s = 1;
                    break;
            }
            robot.bddrivetrain.strafeFor(23 * s, .8 * s, 5);
            sleep(100);
        }
        robot.bddrivetrain.driveFor(20, .8, 4);
    }

    public void stones() {


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

        sleep(500);
        robot.bddrivetrain.driveFor(54,1,10);
        sleep(500);
        robot.bdgrabber.grabUpAuto();
        sleep(500);
        robot.bddrivetrain.driveFor(-22,-1,10);

        telemetry.addData("PATH", "COMPLETE");
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

                default: {
                    return input;
                }
            }
        }

    }
}
