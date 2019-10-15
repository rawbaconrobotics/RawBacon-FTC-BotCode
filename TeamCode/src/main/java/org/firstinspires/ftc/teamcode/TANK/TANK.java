package org.firstinspires.ftc.teamcode.TANK;

import com.qualcomm.robotcore.hardware.HardwareMap;
public class TANK {
    TANKDriveTrain drive;
    //TANKClaw claw;
    //TANKIntake intake;
    TANKLatch latch;
    public TANK(HardwareMap hardwareMap){
        drive = new TANKDriveTrain(hardwareMap);
        //claw = new TANKClaw(hardwareMap);
        //intake = new TANKIntake(hardwareMap);
        latch = new TANKLatch(hardwareMap);
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