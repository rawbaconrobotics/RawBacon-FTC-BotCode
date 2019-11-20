package org.firstinspires.ftc.teamcode.Uhaul;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;



public class Uhaul {
    public UhaulDriveTrain drive;
    public void teleOpActivated() {
        //Activate wheels for opmode
    drive.wheelsTeleOp();
    }

    public Uhaul(LinearOpMode opMode) {
        drive.init();
    }























































































}
