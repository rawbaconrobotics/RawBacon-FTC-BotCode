package org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulComponentImplBase;
/**
 * @author Raw Bacon Coders
 * Defines the uhaul grabber
 */
@Config
public class UhaulGrabber extends UhaulComponentImplBase {
//READY FOR UHAUL!
    String UHAUL_GRABBER_NAME = "uhaul_grabber";

    private ElapsedTime runtime = new ElapsedTime();
    public Servo uhaulGrabber = null;
    public static double UHAUL_GRABBER_OPEN = 0.1;
    public static double UHAUL_GRABBER_CLOSED = 0.6;



    /** Initializes the grabber */
    @Override
    public void init() {
        uhaulGrabber = hardwareMap.servo.get(UHAUL_GRABBER_NAME);
    }
      /** Defines movement controls of the grabber */
    public void moveGrabber() {
        if (gamepad2.x){
            uhaulGrabber.setPosition(UHAUL_GRABBER_OPEN);
        }
        if (gamepad2.y){
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
