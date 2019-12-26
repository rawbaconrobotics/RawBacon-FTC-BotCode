package org.firstinspires.ftc.teamcode.BigDipper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BDDistanceSensor;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BDDriveTrain;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BDGrabber;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BDLatch;

public class Robot {
    //Initialize new components
    public BDDriveTrain bddrivetrain;
    public BDLatch bdlatch;
    public BDGrabber bdgrabber;

    public void teleOpActivated(){
    bddrivetrain.wheelsTeleOp();
    bdlatch.latch();
    bdgrabber.grabber();
    }

    //method to initialize all the stuff for the opmode
 public Robot(LinearOpMode opMode)
 {
     bddrivetrain = new BDDriveTrain(opMode);
     bdlatch = new BDLatch(opMode);
     bdgrabber = new BDGrabber(opMode);
 }
}

