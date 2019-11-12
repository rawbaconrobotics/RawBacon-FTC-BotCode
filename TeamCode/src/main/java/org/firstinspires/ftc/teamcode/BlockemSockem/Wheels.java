package org.firstinspires.ftc.teamcode.BlockemSockem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Clyde_OLD.HardwareHelper;

public class Wheels implements HardwareHelper {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    public void init(HardwareMap mappy){
        leftFront = mappy.get(DcMotor.class, BMSM_HW_Names.LEFTFRONT);
        rightFront = mappy.get(DcMotor.class, BMSM_HW_Names.RIGHTFRONT);
        leftBack = mappy.get(DcMotor.class, BMSM_HW_Names.LEFTBACK);
        rightBack = mappy.get(DcMotor.class, BMSM_HW_Names.RIGHTBACK);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        halt();
    }
    public void halt(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void driveFB(double speed){
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);
    }
}
