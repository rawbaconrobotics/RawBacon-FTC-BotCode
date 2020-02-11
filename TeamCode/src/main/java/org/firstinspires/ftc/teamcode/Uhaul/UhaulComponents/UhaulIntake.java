package org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulComponentImplBase;

import static android.os.SystemClock.sleep;

/**
 * @author Raw Bacon Coders
 * Defines the UhaulIntake proccess 
 */
public class UhaulIntake extends UhaulComponentImplBase {
//READY TO GO FOR INTAKE
    private ElapsedTime runtime = new ElapsedTime();


    String LEFT_INTAKE = "left_intake";
    String RIGHT_INTAKE = "right_intake";

    public DcMotor uhaulLeftIntake = null;
    public DcMotor uhaulRightIntake = null;
    double previousPower = 0;
    public static double intakePower = 1;

    ColorSensor sensorColor = null;

    String COLOR_SENSOR = "color_sensor";

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;


/*
    int blueMinimum = 2;
    int redMinimum = 2; //Print color values to calibrate these before competition!
    int greenMinimum = 2;
    boolean red = false;
    boolean blue = false;
    boolean green = false;

    int reds = 0;
    int blues = 0;
    int greens = 0; */

    /** Initializes the proccess */
    public void init(){
        uhaulLeftIntake = hardwareMap.dcMotor.get(LEFT_INTAKE);
        uhaulRightIntake = hardwareMap.dcMotor.get(RIGHT_INTAKE);

        uhaulLeftIntake.setDirection(DcMotor.Direction.FORWARD);
        uhaulRightIntake.setDirection(DcMotor.Direction.REVERSE);

      //  sensorColor = hardwareMap.colorSensor.get(COLOR_SENSOR);

    }
    /** Runs the intake proccess */
    public void runIntake(){
        if (gamepad2.left_bumper || gamepad2.right_bumper){
            uhaulLeftIntake.setPower(intakePower);
            uhaulRightIntake.setPower(intakePower);
            previousPower = intakePower;
        }
        else if((previousPower != 0)){
            uhaulLeftIntake.setPower(0);
            uhaulRightIntake.setPower(0);
            previousPower = 0;
        }


    }

    public void intakeOFF(){
        uhaulLeftIntake.setPower(1);
        uhaulRightIntake.setPower(1);
    }
    public void intakeON(){
        uhaulLeftIntake.setPower(1);
        uhaulRightIntake.setPower(1);
    }
    /** Defines the proccess for the autonomous */
    public boolean autonomousIntake(){
        runtime.reset();
        boolean stoneFound = true;

        uhaulLeftIntake.setPower(1);
        uhaulRightIntake.setPower(1);

        while((runtime.seconds() < 4) && opModeIsActive()){

            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
            if(( hsvValues[0] < 70 || hsvValues[0] > 50 ) && (hsvValues[2] > 50)){
                sleep(1000);
                uhaulLeftIntake.setPower(0);
                uhaulRightIntake.setPower(0);
                break;
            }


            stoneFound = false;
        }
        uhaulLeftIntake.setPower(0);
        uhaulRightIntake.setPower(0);

        return stoneFound;
    }



    /** Initializes the proccess for the autonomous */
    @Override
    public void initAutonomous() {
        uhaulLeftIntake = hardwareMap.dcMotor.get("left_intake");
        uhaulRightIntake = hardwareMap.dcMotor.get("right_intake");

        uhaulLeftIntake.setDirection(DcMotor.Direction.REVERSE);
        uhaulRightIntake.setDirection(DcMotor.Direction.FORWARD);

        sensorColor = hardwareMap.colorSensor.get(COLOR_SENSOR);

    }

    /** Overrides the default opmode method */
    public UhaulIntake(LinearOpMode opMode) {
        super(opMode);
    }

    //Nice for loop but we aren't using these values (right now)
/*
            for(int i = 0; i<50; i++)
            {
                if(sensorColor.blue() >= blueMinimum)
                    blues++;
                if(sensorColor.red() >= redMinimum)
                    reds++;
            }
            if(sensorColor.green() >= greenMinimum)
                greens++;
            }


            if(reds == 50 && blues == 0)
            {
                red = true;
            }
            else if(blues == 50 && reds == 0)
            {
                blue = true;
            }
            */

}
