package org.firstinspires.ftc.teamcode.Uhaul;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
@TeleOp(name="Vrail Tuner", group="Uhaul")
public class VRailTuner extends LinearOpMode {
    //SLIDER IS READY!
    private ElapsedTime runtime = new ElapsedTime();
    String UHAUL_SLIDER_1 = "uhaul_slider_1";
    String UHAUL_SLIDER_2 = "uhaul_slider_2";
    public CRServo uhaulSlider1 = null; //The slider for the arm
    public CRServo uhaulSlider2 = null;
    public DistanceSensor uhaulSliderDistance = null;

    private static double sliderPower = -1;
    private static double previousPower = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        uhaulSlider1 = hardwareMap.crservo.get(UHAUL_SLIDER_1);
        uhaulSlider2 = hardwareMap.crservo.get(UHAUL_SLIDER_2);
        // uhaulStoneFlipper = hardwareMap.crservo.get("uhaul_stone_flipper");
        // uhaulStoneGrabber = hardwareMap.crservo.get("uhaul_stone_grabber");
        uhaulSliderDistance = hardwareMap.get(DistanceSensor.class, "slider_distance_sensor");

        uhaulSlider1.setDirection(CRServo.Direction.FORWARD);
        uhaulSlider1.setDirection(CRServo.Direction.REVERSE);

        waitForStart();

       while(opModeIsActive()){
        telemetry.addData("inches", uhaulSliderDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("1 power", uhaulSlider1.getPower());
        telemetry.addData("2 power", uhaulSlider2.getPower());
        telemetry.update();

        uhaulSlider1.setPower(gamepad1.left_stick_y * 0.75);
        uhaulSlider2.setPower(gamepad1.left_stick_y * 0.75);
       }
    }

    /** Initializes the proccess */





    public static double distanceFromBase;
    /** Defines the slider movement controls */
    public void moveSlider() {

        distanceFromBase = uhaulSliderDistance.getDistance(DistanceUnit.INCH);

        if(gamepad2.right_trigger > 0.1){
            uhaulSlider1.setPower(-sliderPower);
            uhaulSlider2.setPower(-sliderPower);

            while((distanceFromBase > 5) && (!gamepad2.right_bumper) && (!gamepad2.left_bumper)){
                telemetry.addData("Distance from base", distanceFromBase);
                telemetry.addData("Don't worry, We'll stop at 20 inches away...", "or maybe do worry idk this isn't tested yet");
            }

            uhaulSlider1.setPower(0);
            uhaulSlider2.setPower(0);

        }
        else if(gamepad2.left_trigger > 0.1){
            uhaulSlider1.setPower(sliderPower);
            uhaulSlider2.setPower(sliderPower);

            while((distanceFromBase < 20) && (!gamepad2.right_bumper) && (!gamepad2.left_bumper)){
                telemetry.addData("Distance from base", distanceFromBase);
                telemetry.addData("Don't worry, We'll stop at 20 inches away...", "or maybe do worry idk this isn't tested yet");
            }

            uhaulSlider1.setPower(0);
            uhaulSlider2.setPower(0);

        }
        if(gamepad2.right_bumper || gamepad2.left_bumper){
            uhaulSlider1.setPower(0);
            uhaulSlider2.setPower(0);
            //currently, do nothing
        }


    }
//idea: contPower = -.20;
    /** Defines power values for slider1 and 2 */
    public void slideOut(){
        uhaulSlider1.setPower(1);
        uhaulSlider2.setPower(1);
        while(opModeIsActive() && uhaulSliderDistance.getDistance(DistanceUnit.INCH) < 10){
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
        while(opModeIsActive() && uhaulSliderDistance.getDistance(DistanceUnit.INCH) > 0.5){
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

}

