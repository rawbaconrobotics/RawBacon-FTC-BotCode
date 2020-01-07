package org.firstinspires.ftc.teamcode.BlockemSockem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Wheels implements HardwareHelper {
    //MATH/TESTING NEEDS TO BE DONE!!!!!!!!!!!!!!!!!!!!!!!
    private final double COUNTS_PER_INCH = 600;
    private final double COUNTS_PER_DEGREE = 600;
    private final double STRAFE_COUNTS_PER_INCH = 600;

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private ElapsedTime timer = new ElapsedTime();
    private boolean hasLinearOpMode;
    private LinearOpMode opper;

    public Wheels(){
        hasLinearOpMode = false;
    }
    public Wheels(LinearOpMode oppy){
        opper = oppy;
        hasLinearOpMode = true;
    }

    public void init(HardwareMap mappy){
        leftFront = mappy.get(DcMotor.class, BESE_HW_Names.LEFTFRONT);
        rightFront = mappy.get(DcMotor.class, BESE_HW_Names.RIGHTFRONT);
        leftBack = mappy.get(DcMotor.class, BESE_HW_Names.LEFTBACK);
        rightBack = mappy.get(DcMotor.class, BESE_HW_Names.RIGHTBACK);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        halt();
    }
    //Drive by time/encoder counts won't work without a LinearOpMode
    public boolean opModeIsActive(){
        if(hasLinearOpMode){
            return opper.opModeIsActive();
        }
        else{
            return false;
        }
    }
    public void halt(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
    void setMode(DcMotor.RunMode mode){
        leftFront.setMode(mode);
        rightFront.setMode(mode);
        leftBack.setMode(mode);
        rightBack.setMode(mode);
    }
    public boolean isBusy(){
        return (leftFront.isBusy() && rightFront.isBusy()) &&
                (leftBack.isBusy() && rightBack.isBusy());
    }

    public void resetEncoders(){
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void left_drive(double speed){
        leftFront.setPower(speed);
        leftBack.setPower(speed);
    }
    public void right_drive(double speed){
        rightFront.setPower(speed);
        rightBack.setPower(speed);
    }
    //Positive is forwards
    public void drive(double speed){
        left_drive(speed);
        right_drive(speed);
    }
    //Positive is clockwise
    public void turn(double speed){
        leftFront.setPower(speed);
        rightFront.setPower(-speed);
        leftBack.setPower(speed);
        rightBack.setPower(-speed);
    }
    //Positive is left
    public void strafe(double speed){
        leftFront.setPower(-speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(-speed);
    }
    private void waitFor(double seconds){
        timer.reset();
        while(opModeIsActive() && (timer.seconds() < seconds)){
            //Could put telemetry here.
        }
    }
    public void driveTime(double seconds, double speed){
        drive(speed);
        waitFor(seconds);
        halt();
    }
    public void turnTime(double seconds, double speed){
        turn(speed);
        waitFor(seconds);
        halt();
    }
    public void strafeTime(double seconds, double speed){
        strafe(speed);
        waitFor(seconds);
        halt();
    }

    public void addCounts(double distance, String type){
        int counts;
        int pattern[] = {1, 1, 1, 1};
        switch(type){
            case "drive":
                counts = (int)(distance * COUNTS_PER_INCH);
                break;
            case "turn":
                counts = (int)(distance * COUNTS_PER_DEGREE);
                pattern[1] = -1;
                pattern[3] = -1;
                break;
            case "strafe":
                counts = (int)(distance * STRAFE_COUNTS_PER_INCH);
                pattern[0] = -1;
                pattern[3] = -1;
                break;
            default:
                throw new IllegalArgumentException("Invalid type \""+type+"\"");
        }
        leftFront.setTargetPosition(leftFront.getTargetPosition() + counts * pattern[0]);
        rightFront.setTargetPosition(rightFront.getTargetPosition() + counts * pattern[1]);
        leftBack.setTargetPosition(leftBack.getTargetPosition() + counts * pattern[2]);
        rightBack.setTargetPosition(rightBack.getTargetPosition() + counts * pattern[3]);
    }
    public void driveFor(double inches, double speed){
        addCounts(inches, "drive");
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive(speed);
        while(isBusy() && opModeIsActive()){}
        halt();
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void turnFor(double degrees, double speed){
        addCounts(degrees, "turn");
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turn(speed);
        while(isBusy() && opModeIsActive()){}
        halt();
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void strafeFor(double inches, double speed){
        addCounts(inches, "strafe");
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        strafe(speed);
        while(isBusy() && opModeIsActive()){}
        halt();
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}