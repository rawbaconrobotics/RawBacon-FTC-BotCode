package org.firstinspires.ftc.teamcode.BlockemSockem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Clyde_OLD.HardwareHelper;

public class Wheels implements HardwareHelper {
    //THIS NEEDS TO BE DONE!!!!!!!!!!!!!!!!!!!!!!!
    private final double COUNTS_PER_INCH = 6;
    private final double COUNTS_PER_DEGREE = 6;
    private final double STRAFE_COUNTS_PER_INCH = 6;

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private ElapsedTime timer = new ElapsedTime();

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

    //Positive is forwards
    public void drive(double speed){
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);
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
    public void driveTime(double seconds, double speed){
        timer.reset();
        drive(speed);
        while(/*opModeIsActivate() && */(timer.seconds() < seconds)){}
        halt();
    }
    public void turnTime(double seconds, double speed){
        timer.reset();
        turn(speed);
        while(/*opModeIsActivate() && */(timer.seconds() < seconds)){}
        halt();
    }
    public void strafeTime(double seconds, double speed){
        timer.reset();
        strafe(speed);
        while(/*opModeIsActivate() && */(timer.seconds() < seconds)){}
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
        rightFront.setTargetPosition(rightFront.getTargetPosition() + counts * pattern[0]);
        leftBack.setTargetPosition(leftBack.getTargetPosition() + counts * pattern[0]);
        rightBack.setTargetPosition(rightBack.getTargetPosition() + counts * pattern[0]);
    }
    public void driveFor(double inches, double speed){
        addCounts(inches, "drive");
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive(speed);
        while(isBusy()){}
        halt();
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void turnFor(double degrees, double speed){
        addCounts(degrees, "turn");
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turn(speed);
        while(isBusy()){}
        halt();
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void strafeFor(double inches, double speed){
        addCounts(inches, "strafe");
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        strafe(speed);
        while(isBusy()){}
        halt();
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}