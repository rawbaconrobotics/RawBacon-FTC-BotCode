package org.firstinspires.ftc.teamcode.Clyde;

import com.qualcomm.robotcore.hardware.HardwareMap;

//Holds all of the parts of Clyde in one class
public class FergBot implements HardwareHelper{
    FergieWheels wheels = new FergieWheels();
    FergieArm arm = new FergieArm();
    Latch latch = new Latch();
    public void init(HardwareMap mappy){
        wheels.init(mappy);
        arm.init(mappy);
        latch.init(mappy);
    }
    public void resetEncoders () {
        wheels.resetEncoders();
        arm.resetEncoders();
    }
}
