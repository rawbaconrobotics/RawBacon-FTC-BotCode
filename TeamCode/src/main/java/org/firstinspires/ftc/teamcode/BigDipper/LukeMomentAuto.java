package org.firstinspires.ftc.teamcode.BigDipper;

import android.graphics.Bitmap;
import android.hardware.Camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BDLatch;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BaseLinearOpMode;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
@Config
@Autonomous(name = "LukeMoment Autonomous", group = "Big Dipper")

public class LukeMomentAuto extends BaseLinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //RobotWheelsTest robotWheelsTest = new RobotWheelsTest(this);

    public BDLatch bdlatch;
    private static final boolean PHONE_IS_PORTRAIT = false;


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

    public static double FIRSTdriveinches1 = -39;
    public static double SECONDstrafeinches1 = 7;
    public static double THIRDstrafeinches2 = 4;
    public static double FOURTHdriveinches2 = 25;
    public static double FIFTHstrafeinches3 = 28;
    public static double SIXTHdriveinches3 = -15;
    public static double SEVENTHdriveinches4 = 31.5;
    public static int EIGHTHturndegrees1 = 0;
    public static double NINTHstrafeinches4 = 50;

    private static final double COUNTS_PER_MOTOR_REV = 1120;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
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
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

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
    private static final float quadField = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;



    private boolean shouldWrite = false;


    private Mat bitmapToMat(Bitmap bit, int cvType) {
        Mat newMat = new Mat(bit.getHeight(), bit.getWidth(), cvType);

        Utils.bitmapToMat(bit, newMat);

        return newMat;
    }

    @Override
    public void on_init() {
        System.out.println("INIT PROCESS STARTING");

        robot.robotWheelsTest.initAutonomous();

        robot.bdlatch.initAutonomous();

        robot.bdgrabber.initAutonomous();

        //webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();


        /**
         * We also indicate which camera on the RC we wish to use.
         */
        //parameters.cameraName = webcamName;
//NOT SURE IF WE NEED THE ABOVE!

        //  Instantiate the Vuforia engine


        // FtcDashboard dashboard = FtcDashboard.getInstance();

        //FtcDashboard dashboard = FtcDashboard.getInstance();


        System.out.println("INITIALIZED, STARTING TO LOOK FOR SKYSTONES");



      /*      if (gamepad1.a)
                shouldWrite = true;
            else
                shouldWrite = fals  e; */

    }
    @Override
    public void run() {

        System.out.println("ROBOT RUN SEQUENCE INITIALIZED!");


        //robotWheelsTest.initAutonomous();
        //distanceSensor.init();
        //bdlatch.init();


        runtime.reset();
        telemetry.addData("Runtime Reset", "Complete");
        System.out.println("RUNTIME RESET COMPLETE");
        robot.robotWheelsTest.driveFor(FIRSTdriveinches1,0.5,10);
        sleep(750);
        robot.robotWheelsTest.strafeFor(SECONDstrafeinches1,0.2,true, 10);
        sleep(750);
        robot.robotWheelsTest.strafeFor(THIRDstrafeinches2,0.2,false, 10);
        sleep(750);

        robot.robotWheelsTest.driveFor(FOURTHdriveinches2,0.5,10);
        sleep(750);

        robot.robotWheelsTest.strafeFor(FIFTHstrafeinches3,0.2, true, 10);
        sleep(750);

        robot.robotWheelsTest.driveFor(SIXTHdriveinches3,0.5,10);
        sleep(750);

        robot.bdlatch.closeLatch();
        sleep(750);

        robot.robotWheelsTest.driveFor(SEVENTHdriveinches4,0.5,10);
        sleep(750);

        robot.bdlatch.openLatch();
        sleep(750);

        robot.robotWheelsTest.turnFor(EIGHTHturndegrees1, 0.5, 10);

        robot.robotWheelsTest.strafeFor(NINTHstrafeinches4,0.5,false, 10);


    }


    @Override
    public void on_stop() {
        //do something when the robot stops?
    }


}

