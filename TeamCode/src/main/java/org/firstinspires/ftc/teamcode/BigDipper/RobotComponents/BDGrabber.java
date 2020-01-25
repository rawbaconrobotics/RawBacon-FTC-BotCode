package org.firstinspires.ftc.teamcode.BigDipper.RobotComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static android.os.SystemClock.sleep;

/**
 * Represents the grabber  
 * @author Raw Bacon Coders
 */

public class BDGrabber extends RobotComponentImplBase{
    private final static String GRABBER_SERVO_NAME = "tank_grabber";
    double GRABBER_OPEN = 0.9;
    double GRABBER_CLOSED = 0.2;

    public static boolean grabberButton = false;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private CRServo grabberServo = null;

    
    /**
     * Overrides the opMode method
     */
    public BDGrabber(LinearOpMode opMode) {
        super(opMode);
    }

     /**
     * Initializes the grabber
     */
    @Override
    public void init() {
        grabberServo = hardwareMap.crservo.get(GRABBER_SERVO_NAME);
    }
    
     /**
     * Initializes the grabber
     */

    @Override
    public void initAutonomous() {
        grabberServo = hardwareMap.crservo.get(GRABBER_SERVO_NAME);

        grabberServo.setPower(-0.6);
        sleep(750);
        grabberServo.setPower(0);
    }


     /**
     * Opens or closes the grabber based upon the current state it is in
     */
    public void grabber() {

        boolean openGrabber = gamepad1.right_bumper;
        boolean closeGrabber = gamepad1.left_bumper;
if(opModeIsActive()) {
    if (openGrabber && (grabberServo.getPower()) != 1) {
        grabberServo.setPower(1);
    }
    if (closeGrabber && (grabberServo.getPower() != -0.6)) {
        grabberServo.setPower(-0.6);
    }
    if ((!openGrabber || !closeGrabber) && (grabberServo.getPower()) != 0) {
        grabberServo.setPower(0);
    }
}
    }

    public void grabDownAuto(){
        if(opModeIsActive()) {
            grabberServo.setPower(0.6);
            sleep(1000);
            grabberServo.setPower(0);
        }
    }
    public void grabUpAuto(){
        if(opModeIsActive()) {
            grabberServo.setPower(-0.6);
            sleep(750);
            grabberServo.setPower(0);
        }
    }
}
