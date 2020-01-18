package org.firstinspires.ftc.teamcode.Uhaul;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Uhaul.UhaulLinearOpMode;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static android.os.SystemClock.sleep;



@Autonomous(name= "Uhaul Autonomous (Official)", group="Uhaul")

//
// ******  We need to add all the other components that are used throughout the autonomous!
// ******  Make sure ALL VALUES ARE TESTED AND TUNED, Including PID.
// ******  Then add below in run() what we need to do during auto.
//

public class UhaulAutonomous extends UhaulLinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;

    @Override
    public void on_init() {
        System.out.println("INIT PROCESS STARTING");

        
        robot.drive.initAutonomous();
        robot.latch.initAutonomous();
        robot.lift.initAutonomous();
        robot.grabber.initAutonomous();
        robot.intake.initAutonomous();
        robot.slider.initAutonomous();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Values", valLeft + "   " + valMid + "   " + valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            telemetry.update();
        }

    }
    @Override
    public void run () {
        runtime.reset();

        if (valLeft == 0) { // stone is on left, run left path
            System.out.println("DRIVING LEFT PATH");
            robot.drive.turnFor(-90, 0.5, 15);
            sleep(1000);
//the above needs to go
            robot.drive.driveFor(16, 0.5, 15);
            sleep(1000);
            System.out.println("DRIVING LEFT PATH PART 2");
            robot.drive.turnFor(-90, 0.5, 15);
            sleep(1000);
            robot.drive.driveFor(9, 0.5, 15);
            sleep(1000);
            robot.drive.turnFor(90, 0.5, 15);
            sleep(1000);
            robot.drive.driveFor(18, 0.5, 15);
            sleep(1000);
            robot.drive.turnFor(-180, 0.5, 15);
            sleep(1000);
            robot.drive.driveFor(23.5, 0.5, 15);
            sleep(1000);
            robot.drive.turnFor(90, 0.5, 15);
            sleep(1000);
            robot.drive.driveFor(25.125, 0.5, 15);
            sleep(1000);

        } else if (valMid == 0) { // stone is in middle, run middle path
//move straight approx 34 inches, turn 180 degrees counter clockwise, go forward 47/2 inches, turn 90 deg. clockwise, forward 34.125 inches.
            robot.drive.driveFor(34, 0.5, 15);
            sleep(1000);
            System.out.println("DRIVING MID PATH");

            robot.drive.turnFor(-180, 0.5, 15);
            sleep(1000);
            System.out.println("DRIVING MID PATH PART 2");

            robot.drive.driveFor(23.5, 0.5, 15);
            sleep(1000);
            robot.drive.turnFor(90, 0.5, 15);
            sleep(1000);
            robot.drive.driveFor(34.125, 0.5, 15);
            sleep(1000);




        } else if (valRight == 0) { //stone on right, run right path

            robot.drive.driveFor(16, 0.5, 5);
            sleep(1000);
            System.out.println("DRIVING RIGHT PATH");

            robot.drive.turnFor(90, 0.5, 15);
            sleep(1000);
            System.out.println("DRIVING RIGHT PATH PART 2");

            robot.drive.driveFor(9, 0.5, 15);
            sleep(1000);
            robot.drive.turnFor(-90, 0.5, 15);
            sleep(1000);
            robot.drive.driveFor(18,0.5, 15);
            sleep(1000);
            robot.drive.turnFor(-180, 0.5, 15);
            sleep(1000);
            robot.drive.driveFor(23.5, 0.5, 15);
            sleep(1000);
            robot.drive.turnFor(90, 0.5, 15);
            sleep(1000);
            robot.drive.driveFor(48.125, 0.5, 15);
            sleep(1000);
            robot.drive.driveFor(-5, -0.5, 15);
            sleep(1000);



        } else {
            //whoops it broke
            telemetry.addData("UNABLE TO FIND SKYSTONE", ", GOING TO SKYBRIDGE");
            telemetry.update();
            System.out.println("IT BROKE");

            robot.drive.driveFor(9, 0.5, 15);
            sleep(1000);
            robot.drive.turnFor(-90, 0.5, 15);
            sleep(1000);
            robot.drive.driveFor(47, 0.5, 15);
            sleep(1000);


        }



        telemetry.addData("PATH", "COMPLETE");
        System.out.println("PATH COMPLETE");


        sleep(100);
        //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);

    }


    public void on_stop() {

    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
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
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
}

