package org.firstinspires.ftc.teamcode.BigDipper.RobotComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

 /**
   * Represents the latch
   * @author Raw Bacon Coders
   */

public class BDLatch extends RobotComponentImplBase {

    private final static String LATCH_SERVO_NAME = "bd_latch" ;

    double LATCH_OPEN = 0;
    double LATCH_CLOSED = 0.3;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo bdLatch = null;

    
     /**
     * Overrides the opMode method
     */
    public BDLatch(LinearOpMode opMode) {
        super(opMode);
    }

     /**
     * Initializes the latch
     */
    @Override
    public void init() {
        bdLatch = hardwareMap.servo.get(LATCH_SERVO_NAME);
    }

    
     /**
     * Initializes the latch
     */
    public void initAutonomous(){
        bdLatch = hardwareMap.servo.get(LATCH_SERVO_NAME);
        bdLatch.setPosition(LATCH_OPEN);
    }

     /**
     * Defines latch positions to the gamepad
     */
    public void latch() {
        boolean openLatch = gamepad2.y;
        boolean closeLatch = gamepad2.x;
        if(openLatch){
            bdLatch.setPosition(LATCH_OPEN);
            //latchServo2.setPosition(LATCH_CLOSED);
        }
        if (closeLatch){
            bdLatch.setPosition(LATCH_CLOSED);
            //latchServo2.setPosition(LATCH_OPEN);
        }
    }
    
     /**
     * Opens the latch
     */
    public void openLatch(){
        System.out.println("LATCH OPENING");
        bdLatch.setPosition(LATCH_OPEN);
        //latchServo2.setPosition(LATCH_CLOSED);
    }

     /**
     * Closes the latch
     */
    public void closeLatch(){
        System.out.println("LATCH CLOSING");
        bdLatch.setPosition(LATCH_CLOSED);
        //latchServo2.setPosition(LATCH_OPEN);
    }
}



