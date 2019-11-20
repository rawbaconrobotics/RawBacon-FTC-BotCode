package org.firstinspires.ftc.teamcode.BlockemSockem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Clyde_OLD.HardwareHelper;

class Robot implements HardwareHelper {
    Wheels w = new Wheels();
    Arm a = new Arm();
    Latch l = new Latch();
    public void init(HardwareMap mappy) {
        w.init(mappy);
        a.init(mappy);
        l.init(mappy);
    }
    public void resetEncoders(){
        w.resetEncoders();
        a.resetEncoders();
    }
}
