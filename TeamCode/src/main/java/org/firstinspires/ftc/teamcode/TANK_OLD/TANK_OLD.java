package org.firstinspires.ftc.teamcode.TANK_OLD;

import com.qualcomm.robotcore.hardware.HardwareMap;
public class TANK_OLD {
    TANKDriveTrain_OLD drive;
    //TANKClaw_OLD claw;
    TANKLatch_OLD latch;
    public TANK_OLD(HardwareMap hardwareMap){
        drive = new TANKDriveTrain_OLD(hardwareMap);
        //claw = new TANKClaw_OLD(hardwareMap);
       //latch = new TANKLatch_OLD(hardwareMap);
    }
    public void shutdown(){
        //intake.shutdown();
        drive.halt();
        //claw.TANKStopClaw();
    }
    /*Front and back 16.5 in
    sides are 12 in
     */

}