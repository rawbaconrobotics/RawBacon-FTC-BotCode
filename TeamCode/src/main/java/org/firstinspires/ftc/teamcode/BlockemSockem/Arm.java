package org.firstinspires.ftc.teamcode.BlockemSockem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Clyde_OLD.HardwareHelper;

public class Arm implements HardwareHelper {
    //CHANGE/TEST ThESE !!!!!!!!!!!!!!!!!!
    private final static double CLAW_OPEN = 0;
    private final static double CLAW_CLOSE = 1;
    private final static int ARM_UPPER_BOUND = 1000;
    private final static int ARM_LOWER_BOUND = 0;
    private final static double COUNTS_PER_DEGREE = 3;

    private DcMotor arm;
    private Servo claw;
    private ElapsedTime timer = new ElapsedTime();
    private boolean hasLinearOpMode;
    private LinearOpMode opper;

    public Arm(){
        hasLinearOpMode = false;
    }
    public Arm(LinearOpMode oppy){
        opper = oppy;
        hasLinearOpMode = true;
    }
    public void init(HardwareMap mappy) {
        arm = mappy.get(DcMotor.class, BESE_HW_Names.ARM);
        claw = mappy.get(Servo.class, BESE_HW_Names.ClAW);
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        claw.setPosition(CLAW_CLOSE);
    }
    public boolean opModeIsActive(){
        if(hasLinearOpMode){
            return opper.opModeIsActive();
        }
        //Drive by encoder counts/time simply won't work outside of a LinearOpMode
        else{
            return false;
        }
    }

    public void openClaw(){
        claw.setPosition(CLAW_OPEN);
    }
    public void closeClaw(){
        claw.setPosition(CLAW_CLOSE);
    }

    private void setArmDir(double speed){
        if(speed >= 0){
            arm.setTargetPosition(ARM_UPPER_BOUND);
        }
        else{
            arm.setTargetPosition(ARM_LOWER_BOUND);
        }
    }
    private void moveArm(double speed){
        arm.setPower(speed);
        setArmDir(speed);
    }
    private void stopArm(){
        arm.setPower(0);
    }
    public void moveArmTeleop(double speed){
        moveArm(speed);
        if(!arm.isBusy()) stopArm();
    }
    public void moveArmTime(double seconds, double speed){
        timer.reset();
        moveArm(speed);
        while(timer.seconds() < seconds && opModeIsActive()){
            if(!arm.isBusy()) break;
        }
        stopArm();
    }
    public void moveArmFor(int degrees, double speed){
        int targetDist = arm.getCurrentPosition() + (int) COUNTS_PER_DEGREE * degrees;
        targetDist = Range.clip(targetDist, ARM_LOWER_BOUND, ARM_UPPER_BOUND);
        arm.setTargetPosition(targetDist);
        arm.setPower(speed);
        while(arm.isBusy() && opModeIsActive()){}
        stopArm();
    }

    public void resetEncoders(){
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
