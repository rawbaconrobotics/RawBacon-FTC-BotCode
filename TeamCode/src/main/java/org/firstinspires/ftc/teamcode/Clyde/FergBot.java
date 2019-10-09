package org.firstinspires.ftc.teamcode.Clyde;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Clyde.FergieArm;
import org.firstinspires.ftc.teamcode.Clyde.FergieWheels;

public class FergBot {
    FergieWheels wheels;
    FergieArm arm;
    public FergBot(HardwareMap mappy){
        wheels = new FergieWheels(mappy);
        arm = new FergieArm(mappy);
    }
}
