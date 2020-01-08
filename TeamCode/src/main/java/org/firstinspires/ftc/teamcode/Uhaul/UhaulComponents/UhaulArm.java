package org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulComponentImplBase;

public class UhaulArm extends UhaulComponentImplBase {

    private ElapsedTime runtime = new ElapsedTime();
    public Servo uhaulSlider1 = null; //The slider for the arm
    public Servo uhaulSlider2 = null; //The thing that holds the block and rotates
    public Servo uhaulStoneFlipper = null; //The slider for the arm
    public Servo uhaulStoneGrabber = null; //The thing that holds the block and rotates

    public UhaulArm(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init() {
        uhaulSlider1 = hardwareMap.servo.get("uhaul_slider_1");
        uhaulSlider2 = hardwareMap.servo.get("uhaul_slider_2");
        uhaulStoneFlipper = hardwareMap.servo.get("uhaul_stone_flipper");
        uhaulStoneGrabber = hardwareMap.servo.get("uhaul_stone_grabber");

    }

    public void moveSlider() {
//WORKING ON THIS CODE!!!!!!!!!!!!!
        uhaulSlider1.setPosition(gamepad2.left_stick_y);
        uhaulSlider2.setPosition(gamepad2.left_stick_y);



     //   if (((slider > -0.1) && (slider < 0.1))) {
            //uhaulSlider1.setPower(0);
            //uhaulSlider2.setPower(0);
       // } else {
            //uhaulSlider1.setPower(slider);
            //uhaulSlider2.setPower(slider);
            //slider = sliderPower;
        }


    }

    /*
    public void moveSliderCR() {
        double sliderPower = -1;


        double slider = gamepad2.left_stick_y;

        slider = sliderPower;

        if (((slider > -0.1) && (slider < 0.1))) {
            uhaulSlider1.setPower(0);
            uhaulSlider2.setPower(0);
        } else {
            uhaulSlider1.setPower(slider);
            uhaulSlider2.setPower(slider);
            slider = sliderPower;
        }


    }
*/
    public void sliderAuto() {


    }

    public void rotateAuto() {

    }

    public void grabAuto() {

    }

    @Override
    public void initAutonomous() {

        uhaulSlider1 = hardwareMap.servo.get("uhaul_slider_1");
        uhaulSlider2 = hardwareMap.servo.get("uhaul_slider_2");
        uhaulStoneFlipper = hardwareMap.servo.get("uhaul_stone_flipper");
        uhaulStoneGrabber = hardwareMap.servo.get("uhaul_stone_grabber");

    }
}