package org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulComponentImplBase;
/**
 * @author Raw Bacon Coders
 * Defines the uhaul grabber
 */
public class UhaulGrabber extends UhaulComponentImplBase {
//READY FOR UHAUL!
    String UHAUL_GRABBER_NAME = "uhaul_grabber";

    private ElapsedTime runtime = new ElapsedTime();
    public Servo uhaulGrabber = null;
    private static final double UHAUL_GRABBER_OPEN = 0.8;
    private final static double UHAUL_GRABBER_CLOSED = 0.2;



    /** Initializes the grabber */
    @Override
    public void init() {
        uhaulGrabber = hardwareMap.servo.get(UHAUL_GRABBER_NAME);
    }
      /** Defines movement controls of the grabber */
    public void moveGrabber() {
        if (gamepad2.right_bumper){
            uhaulGrabber.setPosition(UHAUL_GRABBER_OPEN);
        }
        if (gamepad2.left_bumper){
            uhaulGrabber.setPosition(UHAUL_GRABBER_CLOSED);
        }
    }
     /** Opens the grabber. */
    public void openGrabber(){
        uhaulGrabber.setPosition(UHAUL_GRABBER_OPEN);

    }
    /** Closes the grabber */    
    public void closeGrabber(){
        uhaulGrabber.setPosition(UHAUL_GRABBER_CLOSED);

    }
    /** Initializes the autonomous */
    @Override
    public void initAutonomous() {

        uhaulGrabber = hardwareMap.servo.get(UHAUL_GRABBER_NAME);
        uhaulGrabber.setPosition(UHAUL_GRABBER_OPEN);

    }
    public UhaulGrabber(LinearOpMode opMode) {
        super(opMode);
    }


}
