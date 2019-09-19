package org.firstinspires.ftc.teamcode.MaxHerrera.MHSkyStone;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class FergBot {
    FergieWheels wheels;
    FergieArm arm;
    public FergBot(HardwareMap mappy){
        wheels = new FergieWheels(mappy);
        arm = new FergieArm(mappy);
    }
}
