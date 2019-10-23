package org.firstinspires.ftc.teamcode.TANK;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Tank {
    //Initialize new components
    public TankDriveTrain drive;




    public void teleOpActivated(){
        //Activate wheels for opmode
    drive.wheelsTeleOp();




    }

 public Tank(LinearOpMode opMode)
 {
     drive = new TankDriveTrain(opMode);
     drive.init();

 }
}

