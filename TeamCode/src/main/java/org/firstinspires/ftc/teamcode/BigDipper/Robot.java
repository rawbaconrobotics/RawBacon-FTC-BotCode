package org.firstinspires.ftc.teamcode.BigDipper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BDDistanceSensor;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BDLatch;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.RobotWheels;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.RobotWheelsTest;

public class Robot {
    //Initialize new components
    public RobotWheels robotWheels;
    public RobotWheelsTest robotWheelsTest;
    public BDDistanceSensor distanceSensor;
    public BDLatch bdlatch;


//public BDAutonomousv1 bdautono = new BDAutonomousv1();

    //alternative method to init the opmode stuff by passing as a constructor
    /*public void init(OpMode opMode){
        HardwareMap hardwareMap = opMode.hardwareMap;
        robotWheels.init(hardwareMap, telemetry);
        distanceSensor.init(hardwareMap, telemetry);
        bdlatch.init(hardwareMap, telemetry);
    }
*/

    public void teleOpActivated(){

        //Activate wheels for opmode



    robotWheelsTest.wheelsTeleOp();

    //Activate distance sensor for opmode

        //DISTANCE SENSOR CODE
        //distanceSensor.findDistance();

    bdlatch.latch();


    }



    //method to initialize all the stuff for the opmode
 public Robot(LinearOpMode opMode)
 {

     robotWheelsTest = new RobotWheelsTest(opMode);
     //distanceSensor = new BDDistanceSensor(opMode);
     bdlatch = new BDLatch(opMode);







 }
}

