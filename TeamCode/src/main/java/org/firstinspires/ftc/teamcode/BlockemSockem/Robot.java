package org.firstinspires.ftc.teamcode.BlockemSockem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

class Robot implements HardwareHelper {
    Wheels wheels;
    //Arm arm;
    Latch latch = new Latch();
    public Robot(){
        wheels = new Wheels();
        //arm = new Arm();
    }
    public Robot(LinearOpMode oppy){
        wheels = new Wheels(oppy);
        //arm = new Arm(oppy);
    }
    public void init(HardwareMap mappy) {
        wheels.init(mappy);
        //arm.init(mappy);
        latch.init(mappy);
    }
    public void resetEncoders(){
        wheels.resetEncoders();
        //arm.resetEncoders();
    }
}
