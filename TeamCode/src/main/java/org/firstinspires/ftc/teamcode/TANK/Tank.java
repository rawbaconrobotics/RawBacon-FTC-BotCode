package org.firstinspires.ftc.teamcode.TANK;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Tank {
    //Initialize new components
    public TankDriveTrain drive;
    public TankArmAndClaw latch;


    public void teleOpActivated() {
        //Activate wheels for opmode
        drive.wheelsTeleOp();
        latch.moveLatch();


    }



        // Wait for the game to start (driver presses PLAY)

    public void reverseMotors() {
        drive.reverseWheels();
    }
    public void setUpAuto() {
        drive.setUpAuto();
    }

    /*public void encoderDrive(double inches, double lfSpeed, double rfSpeed, double lbSpeed, double rbSpeed, double timeoutS) {
        drive.encoderDrive(inches, lfSpeed, rfSpeed, lbSpeed, rbSpeed, timeoutS);
    }

     */
    public void driveByTime(double lfSpeed, double rfSpeed, double lbSpeed, double rbSpeed, double time){
        drive.runMotors(lfSpeed, rfSpeed, lbSpeed, rbSpeed, time);

    }
    public void autoLatch(double position1, double position2){
        latch.autoLatch(position1, position2);
    }

    public Tank(LinearOpMode opMode) {
        drive = new TankDriveTrain(opMode);
        latch = new TankArmAndClaw(opMode);
        drive.init();
        latch.init();

    }
}

