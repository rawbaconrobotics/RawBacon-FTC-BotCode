package org.firstinspires.ftc.teamcode.BigDipper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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


    //public BDAutonomousv1 bdautono = new BDAutonomousv1();

    //alternative method to init the opmode stuff by passing as a constructor
    /*public void init(OpMode opMode){
        HardwareMap hardwareMap = opMode.hardwareMap;
        robotWheels.init(hardwareMap, telemetry);
        distanceSensor.init(hardwareMap, telemetry);
        bdlatch.init(hardwareMap, telemetry);
    }
*/

    public void teleOpActivated(LinearOpMode opMode){
        //Activate wheels for opmode
    robotWheels.wheelsTeleOp(opMode);

    //Activate distance sensor for opmode
    distanceSensor.findDistance(opMode);

    bdlatch.latch(opMode);


    }


    //method to initialize all the stuff for the opmode
 public Robot(LinearOpMode opMode){
     robotWheels.init(opMode);
     distanceSensor.init(opMode);
     bdlatch.init(opMode);

 }
}

