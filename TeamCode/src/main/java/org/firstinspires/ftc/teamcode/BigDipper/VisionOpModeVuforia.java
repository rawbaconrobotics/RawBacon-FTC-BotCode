package org.firstinspires.ftc.teamcode.BigDipper;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.BigDipper.VisionPipeline;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.BlockingQueue;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
/**
NOTE: I USE VUFORIA TO RUN MY PIPELINE BC EASY OPEN CV DOESNT WORK FOR ME
FIRST TRY TO USE EASY OPEN CV, YOU CAN ACHIEVE BETTER FRAMERATES THIS WAY
*/

//@Config
@Autonomous(name="VisionTest_DoNotUse")
public class VisionOpModeVuforia extends LinearOpMode {
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AR4SUKv/////AAABmfJT0mdDL0ZXm77n6pwEjSAl/uY9odgICh+X3rkIRm97PKW8lGA3hifS2szUrRWrTaWJQ6HfFtNMhZqB8LLhJhV7dBcTxJ/EPJ9jrfU3STZuHUUnQkUgAPqUI1Qou+oCUVkoIU95B5A93sRik42Mc1q9mxr8kdbHw3QiZdnwS84gUC5KTeb/qSOialhFe1dYgRJpVyn/kEJSqiZrGGx24xcDA2n6OmO+3M4D8vOjEw6337Z+TzXXVy3NWj/jNW7TxP8VRt1scu9sEZ3DE7IOHgZHw2byS2INBtrT/yN1cpp2mT6Ffe61Kq3wazvkQEII1wEdRGeAAE9z+toUdUMepCAmi4BmZnP7kOg/RJkO3dN5";
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    VisionPipeline p;

    private boolean shouldWrite = false;
    @Override
    public void runOpMode() {
        p = new VisionPipeline();
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;
//NOT SURE IF WE NEED THE ABOVE!

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

       // FtcDashboard dashboard = FtcDashboard.getInstance();

        Mat frame;
        waitForStart();

        while (opModeIsActive()) {
      /*      if (gamepad1.a)
                shouldWrite = true;
            else
                shouldWrite = false; */
            VuforiaLocalizer.CloseableFrame vuFrame = null;
            if (!vuforia.getFrameQueue().isEmpty()) {
                try {
                    vuFrame = vuforia.getFrameQueue().take();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }

                if (vuFrame == null) continue;

                for (int i = 0; i < vuFrame.getNumImages(); i++) {
                    Image img = vuFrame.getImage(i);
                    if (img.getFormat() == PIXEL_FORMAT.RGB565) {
                        Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
                        bm.copyPixelsFromBuffer(img.getPixels());
                        Mat mat = bitmapToMat(bm, CvType.CV_8UC3);
                        Mat ret = p.processFrame(mat);
                        Bitmap displayBitmap = Bitmap.createBitmap(ret.width(), ret.height(), Bitmap.Config.RGB_565);
                        Utils.matToBitmap(ret, displayBitmap);
                        //dashboard.sendImage(displayBitmap);
                    }
                }
            }
            //dashboard.sendTelemetryPacket(new TelemetryPacket());
            telemetry.addData("position: ", p.getVumarkLeftBoundary());
            telemetry.update();

            int xcoord = p.getVumarkLeftBoundary();

            if (xcoord < 200) { // stone is on left, run left path

            }

            else if (xcoord > 200 && xcoord < 400) { // stone is in middle, run middle path
//move straight approx 34 inches, turn 180 degrees counter clockwise, go forward 47/2 inches, turn 90 deg. clockwise, forward 34.125 inches.

                }
            else if (xcoord > 400) { //stone on right, run right path

                    }
            else{
                //whoops it broke
                telemetry.addData("IT BROKE I'M SORRY -Luke: ", p.getVumarkLeftBoundary());
                telemetry.update();
            }


        }

    }

    private Mat bitmapToMat (Bitmap bit, int cvType) {
        Mat newMat = new Mat(bit.getHeight(), bit.getWidth(), cvType);

        Utils.bitmapToMat(bit, newMat);

        return newMat;
    }

}
