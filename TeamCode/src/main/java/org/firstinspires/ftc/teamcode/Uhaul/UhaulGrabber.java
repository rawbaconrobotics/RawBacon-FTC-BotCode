package org.firstinspires.ftc.teamcode.Uhaul;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulComponentImplBase;

public class UhaulGrabber extends UhaulComponentImplBase {

    private ElapsedTime runtime = new ElapsedTime();
    public Servo uhaulGrabber = null;
    private static final double UHAUL_GRABBER_OPEN = 0.8;
    private final static double UHAUL_GRABBER_CLOSED = 0.2;


    public UhaulGrabber(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init() {
        uhaulGrabber = hardwareMap.servo.get("uhaul_grabber");
        uhaulGrabber.setPosition(UHAUL_GRABBER_OPEN);
    }

    public void moveGrabber() {
        if (gamepad2.right_bumper){
            uhaulGrabber.setPosition(UHAUL_GRABBER_OPEN);
        }
        if (gamepad2.left_bumper){
            uhaulGrabber.setPosition(UHAUL_GRABBER_CLOSED);
        }
    }
}