package org.firstinspires.ftc.teamcode.BigDipper.RobotComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.RobotComponentImplBase;


/**
 * Represents a capstone lever
 * @author Raw Bacon Coders
 */

public class BDCapstone extends RobotComponentImplBase {

    private final static String CAPSTONE_SERVO_NAME = "bd_capstone";
    double CAPSTONE_OPEN = 0;
    double CAPSTONE_CLOSED = 1;
    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private Servo bdCapstone = null;


    /**
     * Constructor
     */
    public BDCapstone(LinearOpMode opMode) {
        super(opMode);
    }

    
     /**
     * Initializes the components 
     */
    @Override
    public void init() {
        bdCapstone = hardwareMap.servo.get(CAPSTONE_SERVO_NAME);
    }
    
     /**
     * Initializes the components and opens
     */

    public void initAutonomous() {
        bdCapstone = hardwareMap.servo.get(CAPSTONE_SERVO_NAME);
        bdCapstone.setPosition(CAPSTONE_OPEN);
    }
    
    

     /**
     * Opens or closes the capstone depending on which state it is currently in
     */
    public void releaseCapstone() {

        boolean openCapstone = gamepad2.right_bumper;
        boolean closeCapstone = gamepad2.left_bumper;

        if(opModeIsActive()) {
            if (openCapstone) {
                bdCapstone.setPosition(CAPSTONE_OPEN);
            }
            if (closeCapstone) {
                bdCapstone.setPosition(CAPSTONE_CLOSED);
            }
        }

    }

    public void openCapstoneAuto(){

        if(opModeIsActive()){bdCapstone.setPosition(CAPSTONE_OPEN);}
    }
    public void closeCapstoneAuto(){
        if(opModeIsActive()){bdCapstone.setPosition(CAPSTONE_CLOSED);}
    }
}
