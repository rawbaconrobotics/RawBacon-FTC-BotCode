package org.firstinspires.ftc.teamcode.Uhaul;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulSlider;
import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulDriveTrain;
import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulGrabber;
import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulIntake;
import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulLatch;
import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulLift;

/**
 * Represents the robot Uhaul as a whole. Organizes the various components.
 * @author Raw Bacon Coders
 */
public class Uhaul {
    public UhaulDriveTrain drive;
    public UhaulLatch latch;
    public UhaulLift lift;
    public UhaulGrabber grabber;
    public UhaulIntake intake;
    public UhaulSlider slider;

    /**
     * Used for teleops, when this function is called on a loop, all components
     * on a teleop are activated to function by controller input.
     */
    public void teleOpActivated() {
    drive.wheelsTeleOp(); //Activate wheels for opmode
    latch.moveLatch();
    lift.liftTeleOp();
    grabber.moveGrabber();
    intake.runIntake();
    slider.moveSlider();

    }
    
    /**
     * Constructor
     * @param opMode The opmode that is being run. Use keyword this.
     * Runs on creation.
     * Is not necessary to include because most teleops initialize components usually by themselves!
     * It's more like a fail-safe than anything else, still init yourselves for autos!
     */
    public Uhaul(LinearOpMode opMode) {
        drive = new UhaulDriveTrain(opMode);
        latch = new UhaulLatch(opMode);
        lift = new UhaulLift(opMode);
        grabber = new UhaulGrabber(opMode);
        intake = new UhaulIntake(opMode);
        slider = new UhaulSlider(opMode);
    }
}
