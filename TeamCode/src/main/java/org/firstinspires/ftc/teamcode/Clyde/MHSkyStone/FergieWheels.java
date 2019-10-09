package org.firstinspires.ftc.teamcode.Clyde.MHSkyStone;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


//Class is designed to handle driving/turning in opmodes for Fergie's robot design
                          //Trying out interfaces
public class FergieWheels implements WheelMethods{

    static final double     COUNTS_PER_MOTOR_REV    = 1440;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    //Get diameter of turning wheels
    static final double     OMNIWHEEL_DIAMETER_INCHES  = 4.0;
    //Find circumference of turning wheels
    static final double     OMNIWHEEL_CIRCUMFERENCE    = OMNIWHEEL_DIAMETER_INCHES * 3.1415;
    //Get distance from center of turning to turning wheels
    static final double     TURNER_TO_CENTER_INCHES    = 10.0;
    //Find the total distance a full spin of the robot covers
    static final double     TURNER_FLOOR_CIRCUMFERENCE = TURNER_TO_CENTER_INCHES * 2 * 3.1415;
    //Get drive gear reduction of turning wheels
    static final double     TURN_DRIVE_GEAR_REDUCTION  = 1.0;
    //Find the number of counts in one turn of the turning wheels
    static final double     COUNTS_PER_TURNER_TURN     = COUNTS_PER_MOTOR_REV * TURN_DRIVE_GEAR_REDUCTION;
    //Find the number of counts in a full spin of the robot
    static final double     COUNTS_PER_FULL_SPIN  = (TURNER_FLOOR_CIRCUMFERENCE / OMNIWHEEL_CIRCUMFERENCE) * COUNTS_PER_TURNER_TURN;
    //Find the number of counts in a degree of a full spin of the robot
    static final double     COUNTS_PER_DEGREE          = COUNTS_PER_FULL_SPIN / 360;

    private DcMotor turner1; //Motor that turns
    private DcMotor turner2; //Second motor that turns
    private DcMotor driver; //Motor that drives forward/backward
    private final String TURN_1_NAME = "turner_motor_1";
    private final String TURN_2_NAME = "turner_motor_2";
    private final String DRIVE_NAME = "drive_motor";

    //Class constructor
    public FergieWheels(HardwareMap mappy){
        turner1 = mappy.get(DcMotor.class, TURN_1_NAME);
        turner2 = mappy.get(DcMotor.class, TURN_2_NAME);
        driver = mappy.get(DcMotor.class, DRIVE_NAME);
        turner1.setPower(0);
        turner2.setPower(0);
        driver.setPower(0);
    }

    public void drive(double speed){
        driver.setPower(speed);
    }
    public void turn(double speed){
        turner1.setPower(speed);
        turner2.setPower(speed);
    }

    //Drive for a specified distance using encoders
    public void driveFor(int distance_inches, double speed){
        int targetDist;
        //Was unable to add check for opMode being active
        targetDist = driver.getCurrentPosition() + (int)(distance_inches * COUNTS_PER_INCH);
        driver.setTargetPosition(targetDist);
        driver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driver.setPower(speed);
        while(driver.isBusy()){}
        driver.setPower(0);
        driver.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Turn for a specified amount of degrees using encoders
    public void turnFor(int degrees, double speed){
        int targetDist1;
        int targetDist2;
        //Was unable to add check for opMode being active
        targetDist1 = turner1.getCurrentPosition() + (int)(degrees * COUNTS_PER_DEGREE);
        targetDist2 = turner2.getCurrentPosition() + (int)(degrees * COUNTS_PER_DEGREE);
        turner1.setTargetPosition(targetDist1);
        turner1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turner1.setPower(speed);
        turner2.setTargetPosition(targetDist2);
        turner2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turner2.setPower(speed);
        while(turner1.isBusy() && turner2.isBusy()){}
        turner1.setPower(0);
        turner1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turner2.setPower(0);
        turner2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}