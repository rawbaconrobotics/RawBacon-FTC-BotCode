package org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulComponentImplBase;

import static android.os.SystemClock.sleep;

/**
 * @author Raw Bacon Coders
 * Defines the UhaulIntake proccess 
 */
@Config
public class UhaulIntake extends UhaulComponentImplBase {
//READY TO GO FOR INTAKE
    private ElapsedTime runtime = new ElapsedTime();


    String LEFT_INTAKE = "left_intake";
    String RIGHT_INTAKE = "right_intake";

    public DcMotor uhaulLeftIntake = null;
    public DcMotor uhaulRightIntake = null;
    public Rev2mDistanceSensor intakeDistance = null;
    public static double STONE_INCHES_AWAY = 5;
    double previousPower = 0;
    public static double intakePower = .5;


    //String DISTANCE_SENSOR = "distance_sensor";

    boolean intakeOutwards = false;




    /** Initializes the proccess */
    public void init(){
        uhaulLeftIntake = hardwareMap.dcMotor.get(LEFT_INTAKE);
        uhaulRightIntake = hardwareMap.dcMotor.get(RIGHT_INTAKE);

        uhaulLeftIntake.setDirection(DcMotor.Direction.FORWARD);
        uhaulRightIntake.setDirection(DcMotor.Direction.REVERSE);

    //    intakeDistance = hardwareMap.get(Rev2mDistanceSensor.class, DISTANCE_SENSOR);


    }

    /** Initializes the proccess for the autonomous */
    @Override
    public void initAutonomous() {
        uhaulLeftIntake = hardwareMap.dcMotor.get("left_intake");
        uhaulRightIntake = hardwareMap.dcMotor.get("right_intake");

        uhaulLeftIntake.setDirection(DcMotor.Direction.REVERSE);
        uhaulRightIntake.setDirection(DcMotor.Direction.FORWARD);

       // intakeDistance = hardwareMap.get(Rev2mDistanceSensor.class, DISTANCE_SENSOR);


    }
    /** Runs the intake proccess */
    public void runIntake(){

    //    if((gamepad1.right_trigger > 0.1) && !UhaulLift.liftIsBusy && (intakeDistance.getDistance(DistanceUnit.INCH)) > STONE_INCHES_AWAY){
    //            uhaulLeftIntake.setPower(intakePower);
    //            uhaulRightIntake.setPower(intakePower);
    if(gamepad1.right_trigger > 0.1){
        uhaulLeftIntake.setPower(0.5);
        uhaulRightIntake.setPower(0.5);
        }


    if((gamepad1.left_trigger > 0.1)){
            uhaulLeftIntake.setPower(-0.25);
            uhaulRightIntake.setPower(-0.25);
        }

    if(gamepad1.x){

        uhaulLeftIntake.setPower(0);
        uhaulRightIntake.setPower(0);

         }


      //   if((intakeDistance.getDistance(DistanceUnit.INCH) < STONE_INCHES_AWAY) && (previousPower != 0)) {

        //     uhaulLeftIntake.setPower(0);
          //   uhaulRightIntake.setPower(0);

           // previousPower = 0;
        //}

    }

    public void intakeOFF(){
        uhaulLeftIntake.setPower(1);
        uhaulRightIntake.setPower(1);
    }
    public void intakeON(){
        uhaulLeftIntake.setPower(1);
        uhaulRightIntake.setPower(1);
    }


    /** Overrides the default opmode method */
    public UhaulIntake(LinearOpMode opMode) {
        super(opMode);
    }


}
