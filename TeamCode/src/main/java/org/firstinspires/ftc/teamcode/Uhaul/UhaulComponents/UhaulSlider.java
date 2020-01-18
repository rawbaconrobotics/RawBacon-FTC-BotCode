package org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulComponentImplBase;

import static android.os.SystemClock.sleep;

public class UhaulSlider extends UhaulComponentImplBase {
//SLIDER IS READY!
        private ElapsedTime runtime = new ElapsedTime();
        String UHAUL_SLIDER_1 = "uhaul_slider_1";
        String UHAUL_SLIDER_2 = "uhaul_slider_2";
        public CRServo uhaulSlider1 = null; //The slider for the arm
        public CRServo uhaulSlider2 = null; //The thing that holds the block and rotates

        private static double sliderPower = -1;
        private static double previousPower = 0;


        public UhaulSlider(LinearOpMode opMode) {
            super(opMode);
        }


        @Override
        public void init() {
            uhaulSlider1 = hardwareMap.crservo.get(UHAUL_SLIDER_1);
            uhaulSlider2 = hardwareMap.crservo.get(UHAUL_SLIDER_2);
           // uhaulStoneFlipper = hardwareMap.crservo.get("uhaul_stone_flipper");
           // uhaulStoneGrabber = hardwareMap.crservo.get("uhaul_stone_grabber");

        }

        public void moveSlider() {

            sliderPower = gamepad2.left_stick_y;


            if(((sliderPower > -0.1) && (sliderPower < 0.1)) && (previousPower != sliderPower) ){
                uhaulSlider1.setPower(0);
                uhaulSlider2.setPower(0);
                previousPower = 0;
            }
            else if(previousPower != sliderPower){
                uhaulSlider1.setPower(sliderPower);
                uhaulSlider2.setPower(sliderPower);
                previousPower = sliderPower;
            }
            else{
                //currently, do nothing
            }


        }
//idea: contPower = -.20;
        public void slideOut(){
            uhaulSlider1.setPower(1);
            uhaulSlider2.setPower(1);
            sleep(2000);
            uhaulSlider1.setPower(0);
            uhaulSlider2.setPower(0);
        }
        public void slideIn(){
            uhaulSlider1.setPower(-1);
            uhaulSlider2.setPower(-1);
            sleep(2000);
            uhaulSlider1.setPower(0);
            uhaulSlider2.setPower(0);
        }
      /*  public void rotateAuto(){

        }

        public void grabAuto(){

        }

       */

        @Override
        public void initAutonomous() {

            uhaulSlider1 = hardwareMap.crservo.get("uhaul_slider_1");
            uhaulSlider2 = hardwareMap.crservo.get("uhaul_slider_2");
           // uhaulStoneFlipper = hardwareMap.crservo.get("uhaul_stone_flipper");
           // uhaulStoneGrabber = hardwareMap.crservo.get("uhaul_stone_grabber");

        }
    }