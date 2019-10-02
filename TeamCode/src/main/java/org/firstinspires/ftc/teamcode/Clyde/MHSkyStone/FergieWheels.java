package org.firstinspires.ftc.teamcode.Clyde.MHSkyStone;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


//Class is designed to handle driving/turning in opmodes for Fergie's robot design
                          //Trying out interfaces
public class FergieWheels implements WheelMethods{

    private static final double     COUNTS_PER_MOTOR_REV    = 1440;
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0;
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0;
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    //Get diameter of turning wheels
    private static final double     OMNIWHEEL_DIAMETER_INCHES  = 4.0;
    //Find circumference of turning wheels
    private static final double     OMNIWHEEL_CIRCUMFERENCE    = OMNIWHEEL_DIAMETER_INCHES * 3.1415;
    //Get distance from center of turning to turning wheels
    private static final double     TURNER_TO_CENTER_INCHES    = 12.0; //CHANGE
    //Find the total distance a full spin of the robot covers
    private static final double     TURNER_FLOOR_CIRCUMFERENCE = TURNER_TO_CENTER_INCHES * 2 * 3.1415;
    //Get drive gear reduction of turning wheels
    private static final double     TURN_DRIVE_GEAR_REDUCTION  = 1.0;
    //Find the number of counts in one turn of the turning wheels
    private static final double     COUNTS_PER_TURNER_TURN     = COUNTS_PER_MOTOR_REV * TURN_DRIVE_GEAR_REDUCTION;
    //Find the number of counts in a full spin of the robot
    private static final double     COUNTS_PER_FULL_SPIN  = (TURNER_FLOOR_CIRCUMFERENCE / OMNIWHEEL_CIRCUMFERENCE) * COUNTS_PER_TURNER_TURN;
    //Find the number of counts in a degree of a full spin of the robot
    private static final double     COUNTS_PER_DEGREE          = COUNTS_PER_FULL_SPIN / 360;

    private DcMotor turner; //Motor that turns back
    private DcMotor driver; //Motor that turns wheels/treads in middle
    private final String TURN_NAME = "drivy_boi";
    private final String DRIVE_NAME = "turny_boi";

    //Class constructor
    public FergieWheels(HardwareMap mappy){
        turner = mappy.get(DcMotor.class, TURN_NAME);
        driver = mappy.get(DcMotor.class, DRIVE_NAME);
        turner.setPower(0);
        driver.setPower(0);
    }

    public void drive(double speed){
        driver.setPower(speed);
    }
    public void turn(double speed){
        driver.setPower(speed);
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
        int targetDist;
        //Was unable to add check for opMode being active
        targetDist = turner.getCurrentPosition() + (int)(degrees * COUNTS_PER_DEGREE);
        turner.setTargetPosition(targetDist);
        turner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turner.setPower(speed);
        while(turner.isBusy()){}
        turner.setPower(0);
        turner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetEncoders(){
        driver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driver.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}