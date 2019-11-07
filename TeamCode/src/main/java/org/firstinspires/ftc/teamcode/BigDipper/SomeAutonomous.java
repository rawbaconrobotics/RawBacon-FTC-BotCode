package org.firstinspires.ftc.teamcode.BigDipper;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BDLatch;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BaseLinearOpMode;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.RobotComponentImplBase;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.RobotWheels;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.RobotWheelsTest;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BDLatch.latchButton;

@TeleOp(name="Autonomousv1-Blue-PLACEbotBYpikto ", group="Big Dipper")

public class SomeAutonomous extends BaseLinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    //RobotWheelsTest robotWheelsTest = new RobotWheelsTest(this);

    public BDLatch bdlatch;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;


    private final static String FRONTRIGHT_WHEEL_NAME = "right_drive_front";
    private final static String FRONTLEFT_WHEEL_NAME = "left_drive_front";
    private final static String BACKRIGHT_WHEEL_NAME = "right_drive_back";
    private final static String BACKLEFT_WHEEL_NAME = "left_drive_back";

    private static final double SLOW_DRIVE_SCALAR = 0.2;
    private static final double STICK_DIGITAL_THRESHOLD = 0.25;
    private static final double TURNING_SCALAR = 0.875;
    final double WHEEL_ACCEL_SPEED_PER_SECOND_STRAIGHT = 2;
    final double WHEEL_DECEL_SPEED_PER_SECOND_STRAIGHT = 15;
    final double WHEEL_ACCEL_SPEED_PER_SECOND_TURNING = 15;
    final double WHEEL_DECEL_SPEED_PER_SECOND_TURNING = 15;
    final double WHEEL_MINIMUM_POWER = 0.3; //Allows for deadband compensation.
    final double WHEEL_MAXIMUM_POWER = 1.0;
    public static boolean DONT_RESET_RUNTIME = false;

    private static final double   COUNTS_PER_MOTOR_REV    = 1440;
    private static final double   DRIVE_GEAR_REDUCTION    = 1.0;
    private static final double   WHEEL_DIAMETER_INCHES   = 4.0;
    private static final double   COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    //Get diameter of turning wheels
    /* private static final double   OMNIWHEEL_DIAMETER_INCHES  = 4.0;
    //Find circumference of turning wheels
    private static final double   OMNIWHEEL_CIRCUMFERENCE    = OMNIWHEEL_DIAMETER_INCHES * Math.PI;
    //Get distance from center of turning to turning wheels
    private static final double TURNER_TO_DRIVER_INCHES = 9.5;
    //Find the total distance a full spin of the robot covers
    private static final double   TURNER_FLOOR_CIRCUMFERENCE = TURNER_TO_DRIVER_INCHES * 2 * Math.PI;
    //Get drive gear reduction of turning wheels
    private static final double   TURN_DRIVE_GEAR_REDUCTION  = 1.0;
    //Find the number of counts in one turn of the turning wheels
    private static final double   COUNTS_PER_TURNER_TURN     = COUNTS_PER_MOTOR_REV * TURN_DRIVE_GEAR_REDUCTION;
    //Find the number of counts in a full spin of the robot
    private static final double   COUNTS_PER_FULL_TURN = (TURNER_FLOOR_CIRCUMFERENCE / OMNIWHEEL_CIRCUMFERENCE) * COUNTS_PER_TURNER_TURN;
    //Find the number of counts in a degree of a full spin of the robot
    private static final double   COUNTS_PER_DEGREE          = COUNTS_PER_FULL_TURN / 360;
    */




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
    int xcoord;
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


    private Mat bitmapToMat (Bitmap bit, int cvType) {
        Mat newMat = new Mat(bit.getHeight(), bit.getWidth(), cvType);

        Utils.bitmapToMat(bit, newMat);

        return newMat;
    }

    @Override
    public void on_init() {

        robot.robotWheelsTest.initAutonomous();

        robot.bdlatch.initAutonomous();

        p = new VisionPipeline();
        //webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

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
        //parameters.cameraName = webcamName;
//NOT SURE IF WE NEED THE ABOVE!

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

        // FtcDashboard dashboard = FtcDashboard.getInstance();

        Mat frame;


        VuforiaLocalizer.CloseableFrame vuFrame = null;

        while(!opModeIsActive()){

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

        xcoord = p.getVumarkLeftBoundary();

    }
      /*      if (gamepad1.a)
                shouldWrite = true;
            else
                shouldWrite = false; */




    }

    @Override
    public void run() {




        //robotWheelsTest.initAutonomous();
        //distanceSensor.init();
        //bdlatch.init();



        runtime.reset();
        telemetry.addData("Runtime Reset", "Complete");




            if (0 < xcoord && xcoord < 200) { // stone is on left, run left path
                robot.robotWheelsTest.driveFor(16, 0.5);
                robot.robotWheelsTest.turnFor(-90, 0.5);
                robot.robotWheelsTest.driveFor(9, 0.5);
                robot.robotWheelsTest.turnFor(90, 0.5);
                robot.robotWheelsTest.driveFor(18, 0.5);
                robot.robotWheelsTest.turnFor(-180, 0.5);
                robot.robotWheelsTest.driveFor(23.5, 0.5);
                robot.robotWheelsTest.turnFor(90, 0.5);
                robot.robotWheelsTest.driveFor(25.125, 0.5);

            } else if (xcoord > 200 && xcoord < 400) { // stone is in middle, run middle path
//move straight approx 34 inches, turn 180 degrees counter clockwise, go forward 47/2 inches, turn 90 deg. clockwise, forward 34.125 inches.
                robot.robotWheelsTest.driveFor(34, 0.5);
                robot.robotWheelsTest.turnFor(-180, 0.5);
                robot.robotWheelsTest.driveFor(23.5, 0.5);
                robot.robotWheelsTest.turnFor(90, 0.5);
                robot.robotWheelsTest.driveFor(34.125, 0.5);




            } else if (xcoord > 400) { //stone on right, run right path

                robot.robotWheelsTest.driveFor(16, 0.5);
                robot.robotWheelsTest.turnFor(90, 0.5);
                robot.robotWheelsTest.driveFor(9, 0.5);
                robot.robotWheelsTest.turnFor(-90, 0.5);
                robot.robotWheelsTest.driveFor(18,0.5);
                robot.robotWheelsTest.turnFor(-180, 0.5);
                robot.robotWheelsTest.driveFor(23.5, 0.5);
                robot.robotWheelsTest.turnFor(90, 0.5);
                robot.robotWheelsTest.driveFor(48.125, 0.5);
                robot.robotWheelsTest.driveFor(-5, -0.5);



            } else {
                //whoops it broke
                telemetry.addData("IT BROKE I'M SORRY -Luke ", p.getVumarkLeftBoundary());
                telemetry.update();
                robot.robotWheelsTest.driveFor(9, 0.5);
                robot.robotWheelsTest.turnFor(-90, 0.5);
                robot.robotWheelsTest.driveFor(47, 0.5);


            }



        telemetry.addData("PATH", "COMPLETE");


    }


    @Override
    public void on_stop() {
        //do something when the robot stops?
    }


}
