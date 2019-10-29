package org.firstinspires.ftc.teamcode.Clyde;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


//Class is designed to handle driving/turning in opmodes for Fergie's robot design
                          //Trying out interfaces
public class FergieWheels/* implements WheelMethods */{

    private static final double   COUNTS_PER_MOTOR_REV    = 1440;
    private static final double   DRIVE_GEAR_REDUCTION    = 1.0;
    private static final double   WHEEL_DIAMETER_INCHES   = 4.0;
    private static final double   COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    //Get diameter of turning wheels
    private static final double   OMNIWHEEL_DIAMETER_INCHES  = 4.0;
    //Find circumference of turning wheels
    private static final double   OMNIWHEEL_CIRCUMFERENCE    = OMNIWHEEL_DIAMETER_INCHES * 3.1415;
    //Get distance from center of turning to turning wheels
    private static final double TURNER_TO_DRIVER_INCHES = 9.5;
    //Find the total distance a full spin of the robot covers
    private static final double   TURNER_FLOOR_CIRCUMFERENCE = TURNER_TO_DRIVER_INCHES * 2 * 3.1415;
    //Get drive gear reduction of turning wheels
    private static final double   TURN_DRIVE_GEAR_REDUCTION  = 1.0;
    //Find the number of counts in one turn of the turning wheels
    private static final double   COUNTS_PER_TURNER_TURN     = COUNTS_PER_MOTOR_REV * TURN_DRIVE_GEAR_REDUCTION;
    //Find the number of counts in a full spin of the robot
    private static final double   COUNTS_PER_FULL_TURN = (TURNER_FLOOR_CIRCUMFERENCE / OMNIWHEEL_CIRCUMFERENCE) * COUNTS_PER_TURNER_TURN;
    //Find the number of counts in a degree of a full spin of the robot
    private static final double   COUNTS_PER_DEGREE          = COUNTS_PER_FULL_TURN / 360;

    private DcMotor turner1; //Motor that turns the back wheels
    private DcMotor turner2; //Second motor that turns back wheels
    private DcMotor driver;  //Motor that turns the wheels/treads in the middle
    private final String TURN_NAME = "turner_motor_1";
    private final String TURN_NAME_2 = "turner_motor_2";
    private final String DRIVE_NAME = "driver_motor";
    private LinearOpMode opper;
    private boolean hasOpMode = false;

    //Maps wheel motors/servos
    public void init(HardwareMap mappy) {
        turner1 = mappy.dcMotor.get(TURN_NAME);
        turner2 = mappy.dcMotor.get(TURN_NAME_2);
        driver = mappy.dcMotor.get(DRIVE_NAME);
        turner1.setPower(0);
        turner2.setPower(0);
        driver.setPower(0);
    }
    //FergieWheels class can be set to have an opMode
    public void init(HardwareMap Goodone, LinearOpMode Betterone){
        init(Goodone);
        opper = Betterone;
        hasOpMode = true;
    }
    //Returns true if the current opMode is active or there isn't an opMode
    private boolean opModeIsActivate(){
        if(hasOpMode){
            return opper.opModeIsActive();
        }
        else{
            return true;
        }
    }

    //Set speeds of turning or driving motors
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
        drive(speed);
        while(driver.isBusy() && opModeIsActivate()){}
        drive(0);
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
        turner2.setTargetPosition(targetDist2);
        turner2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turn(speed);
        while((turner1.isBusy() || turner2.isBusy()) && opModeIsActivate()){}
        turn(0);
        turner1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turner2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //I don't know if this needs to be used, but it's here
    public void resetEncoders(){
        driver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turner1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turner2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driver.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turner1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turner2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}