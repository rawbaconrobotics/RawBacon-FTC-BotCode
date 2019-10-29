package org.firstinspires.ftc.teamcode.TANK;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BDDistanceSensor;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BDLatch;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.RobotWheels;

public class Tank {
    //Initialize new components
    public TankDriveTrain drive;

    public void teleOpActivated(){
        //Activate wheels for opmode
    drive.wheelsTeleOp();
    }

    //method to initialize all the stuff for the opmode
 public Robot(LinearOpMode opMode)
 {
     drive = new TankDriveTrain(opMode);
     drive.init();
 }
}

