package org.firstinspires.ftc.teamcode.Clyde;

import com.qualcomm.robotcore.hardware.HardwareMap;


public class FergBot {
    FergieWheels wheels;
    FergieArm arm;
    public FergBot(){
        wheels = new FergieWheels();
        arm = new FergieArm();
    }
    public void init(HardwareMap mappy){
        wheels.init(mappy);
        arm.init(mappy);
    }
    public void resetEncoders () {
        wheels.resetEncoders();
        arm.resetEncoders();
    }
}
