package org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulComponentImplBase;

import static android.os.SystemClock.sleep;


/**
  * @author Raw Bacon Coders
  * Defines the Slider on the Uhaul robot
  */
public class UhaulSlider extends UhaulComponentImplBase {
//SLIDER IS READY!
        private ElapsedTime runtime = new ElapsedTime();
        String UHAUL_SLIDER_1 = "uhaul_slider_1";
        String UHAUL_SLIDER_2 = "uhaul_slider_2";
        public CRServo uhaulSlider1 = null; //The slider for the arm
        public CRServo uhaulSlider2 = null;
        public Rev2mDistanceSensor uhaulSliderDistance = null;

        private static double sliderPower = -1;
        private static double previousPower = 0;


        public UhaulSlider(LinearOpMode opMode) {
            super(opMode);
        }


        /** Initializes the proccess */
        @Override
        public void init() {
            uhaulSlider1 = hardwareMap.crservo.get(UHAUL_SLIDER_1);
            uhaulSlider2 = hardwareMap.crservo.get(UHAUL_SLIDER_2);
           // uhaulStoneFlipper = hardwareMap.crservo.get("uhaul_stone_flipper");
           // uhaulStoneGrabber = hardwareMap.crservo.get("uhaul_stone_grabber");
            uhaulSliderDistance = hardwareMap.get(Rev2mDistanceSensor.class, "slider_distance_sensor");


        }

        public static double distanceFromBase;
        /** Defines the slider movement controls */
        public void moveSlider() {

            distanceFromBase = uhaulSliderDistance.getDistance(DistanceUnit.INCH);

            if(gamepad1.x){
                uhaulSlider1.setPower(-sliderPower);
                uhaulSlider2.setPower(-sliderPower);

                while(distanceFromBase > 5){
                    telemetry.addData("Distance from base", distanceFromBase);
                    telemetry.addData("Don't worry, We'll stop at 5 inches away...", "or maybe do worry idk this isn't tested yet");
                }

                uhaulSlider1.setPower(0);
                uhaulSlider2.setPower(0);

            }
            else if(gamepad1.y){
                uhaulSlider1.setPower(sliderPower);
                uhaulSlider2.setPower(sliderPower);

                while(distanceFromBase < 20){
                    telemetry.addData("Distance from base", distanceFromBase);
                    telemetry.addData("Don't worry, We'll stop at 20 inches away...", "or maybe do worry idk this isn't tested yet");
                }

                uhaulSlider1.setPower(0);
                uhaulSlider2.setPower(0);

            }
            else{
                //currently, do nothing
            }


        }
//idea: contPower = -.20;
        /** Defines power values for slider1 and 2 */
        public void slideOut(){
            uhaulSlider1.setPower(1);
            uhaulSlider2.setPower(1);
while(uhaulSliderDistance.getDistance(DistanceUnit.INCH) < 10){
    telemetry.addData("slider", "sliding out!");
    telemetry.addData("current inches", uhaulSliderDistance.getDistance(DistanceUnit.INCH));
    telemetry.update();
            }

            uhaulSlider1.setPower(0);
            uhaulSlider2.setPower(0);
        }
        /** Defines power values for slider 1 and 2 */
        public void slideIn(){
            uhaulSlider1.setPower(-1);
            uhaulSlider2.setPower(-1);
            while(uhaulSliderDistance.getDistance(DistanceUnit.INCH) > 0.5){
                telemetry.addData("slider", "sliding out!");
                telemetry.addData("current inches", uhaulSliderDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
            uhaulSlider1.setPower(0);
            uhaulSlider2.setPower(0);
        }
      /*  public void rotateAuto(){

        }

        public void grabAuto(){

        }

       */

        /** Initializes the slider for the autonomous */
        @Override
        public void initAutonomous() {

            uhaulSlider1 = hardwareMap.crservo.get("uhaul_slider_1");
            uhaulSlider2 = hardwareMap.crservo.get("uhaul_slider_2");
           // uhaulStoneFlipper = hardwareMap.crservo.get("uhaul_stone_flipper");
           // uhaulStoneGrabber = hardwareMap.crservo.get("uhaul_stone_grabber");
            uhaulSliderDistance = (Rev2mDistanceSensor) hardwareMap.get(DistanceSensor.class, "sensor_range");

        }
    }
