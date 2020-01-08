package org.firstinspires.ftc.teamcode.Uhaul;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulArm;
import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulDriveTrain;
import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulGrabber;
import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulIntake;
import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulLatch;
import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulLift;


public class Uhaul {
    public UhaulDriveTrain drive;
    public UhaulLatch latch;
    public UhaulLift lift;
    public UhaulArm arm;
    public UhaulGrabber grabber;
    public UhaulIntake intake;

    /**
     * Organizes and runs all components
     */
    public void teleOpActivated() {
    drive.wheelsTeleOp(); //Activate wheels for opmode
    latch.moveLatch();
    lift.moveLift();
    arm.moveSlider();
    grabber.moveGrabber();
    intake.runIntake();
    }
    
    /**
     * Constructor
     * @param opMode The opmode that is being run. Use keyword this.
     */
    public Uhaul(LinearOpMode opMode) {
        drive.init();
        latch.init();
        lift.init();
        arm.init();
        grabber.init();
        intake.init();
    }
}
