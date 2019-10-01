package org.firstinspires.ftc.teamcode.BigDipper;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.RobotWheels;

public class Robot {
    public RobotWheels robotWheels = new RobotWheels();
    public void init(HardwareMap hardwareMap, boolean resetEncoders){
        robotWheels.init(hardwareMap);
    }

    public void teleOpActivated(){
    RobotWheels actTele = new RobotWheels();
    actTele.wheelsTeleOp();

    }
}

