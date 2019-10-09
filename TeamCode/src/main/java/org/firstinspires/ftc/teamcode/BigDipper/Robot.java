package org.firstinspires.ftc.teamcode.BigDipper;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BDDistanceSensor;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BDLatch;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.RobotWheels;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    //Initialize new components
    public RobotWheels robotWheels = new RobotWheels();
    public BDDistanceSensor distanceSensor = new BDDistanceSensor();
    public BDLatch bdlatch = new BDLatch();

    Telemetry telemetry;

    //This class initializes all the things for opmode and autonomous!
    public void init(HardwareMap hardwareMap, boolean resetEncoders){
        robotWheels.init(hardwareMap, telemetry);
        distanceSensor.init(hardwareMap, telemetry);
        bdlatch.init(hardwareMap, telemetry);
    }

    public void teleOpActivated(){
        //Activate wheels for opmode
    robotWheels.wheelsTeleOp();

    //Activate distance sensor for opmode
    distanceSensor.telemetryOpModeActive();

    bdlatch.latch();

    }
}

