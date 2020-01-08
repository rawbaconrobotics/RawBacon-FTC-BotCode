package org.firstinspires.ftc.teamcode.BigDipper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BDCapstone;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BDDriveTrain;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BDGrabber;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BDLatch;

/**
 * Organizes the various components on the robot
 * @author Luke Aschenbrener
 */
public class Robot {
    //Initialize new components
    public BDDriveTrain bddrivetrain;
    public BDLatch bdlatch;
    public BDGrabber bdgrabber;
    public BDCapstone bdcapstone;

    /**
     * Runs the teleop on all components
     */
    public void teleOpActivated(){
        bddrivetrain.wheelsTeleOp();
        bdlatch.latch();
        bdgrabber.grabber();
        bdcapstone.releaseCapstone();
    }

    /**
     * Constructor
     */
    public Robot(LinearOpMode opMode){
        bddrivetrain = new BDDriveTrain(opMode);
        bdlatch = new BDLatch(opMode);
        bdgrabber = new BDGrabber(opMode);
        bdcapstone = new BDCapstone(opMode);
    }
}
