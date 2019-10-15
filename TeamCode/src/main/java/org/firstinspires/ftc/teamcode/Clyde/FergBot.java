package org.firstinspires.ftc.teamcode.Clyde;

import com.qualcomm.robotcore.hardware.HardwareMap;


public class FergBot {
    FergieWheels wheels;
    FergieArm arm;
    public FergBot(HardwareMap mappy){
        wheels = new FergieWheels(mappy);
        arm = new FergieArm(mappy);
    }
    public void resetEncoders () {
        wheels.resetEncoders();
        arm.resetEncoders();
    }
}
