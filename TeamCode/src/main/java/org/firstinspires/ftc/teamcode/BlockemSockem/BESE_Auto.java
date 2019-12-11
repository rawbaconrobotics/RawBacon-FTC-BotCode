package org.firstinspires.ftc.teamcode.BlockemSockem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class BESE_Auto extends LinearOpMode {
    Robot bese = new Robot(this);
    @Override
    public void runOpMode() {
        bese.init(hardwareMap);
        bese.resetEncoders();
        waitForStart();
        //  Auto for BESE, blue build side
        //  Basically the same as current TANK movements
        // Movements:
        //Go forward until in line with foundation
        //Strafe left to push foundation into wall
        //Strafe right to previous position
        //Go back until in front of foundation x-wise
        //Strafe left until in middle of foundation y-wise
        //Turn 180 degrees
        //Move back into foundation
        //Move latches down
        //Move forward until just in front of left wall
        //Strafe left until under blue bridge

    }
}
