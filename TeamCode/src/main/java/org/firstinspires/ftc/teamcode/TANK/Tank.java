package org.firstinspires.ftc.teamcode.TANK;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Tank {
    //Initialize new components
    public TankDriveTrain drive;
    public TankArmAndClaw latch;


    public void teleOpActivated() {
        //Activate wheels for opmode
        drive.wheelsTeleOp();
        latch.moveLatch();


    }

    public void driveForward(double distance, double speed){
        drive.encoderDrive(distance, speed, speed, speed, speed);
    }

    public void strafeRight(double distance, double speed){
        drive.encoderDrive(distance, speed, -speed, -speed, speed);
    }

    public Tank(LinearOpMode opMode) {
        drive = new TankDriveTrain(opMode);
        latch = new TankArmAndClaw(opMode);
        drive.init();
        latch.init();

    }
}

