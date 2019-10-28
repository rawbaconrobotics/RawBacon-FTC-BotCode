package org.firstinspires.ftc.teamcode.BigDipper.RobotComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;
import static org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.RobotWheels.BACKLEFT_WHEEL_NAME;
import static org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.RobotWheels.BACKRIGHT_WHEEL_NAME;
import static org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.RobotWheels.FRONTLEFT_WHEEL_NAME;
import static org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.RobotWheels.FRONTRIGHT_WHEEL_NAME;

public class Drivetrain {

    private static final int ENCODER_PORT_1 = 1;
    private static final int ENCODER_PORT_2 = 2;
    private static final double WHEEL_ACCEL_SPEED_PER_SECOND_STRAIGHT = 0.8;
    private static final double WHEEL_DECEL_SPEED_PER_SECOND_STRAIGHT = 15;
    private static final double WHEEL_ACCEL_SPEED_PER_SECOND_TURNING = 15;
    private static final double WHEEL_DECEL_SPEED_PER_SECOND_TURNING = 15;
    private static final double WHEEL_MINIMUM_POWER = 0.3; //Allows for deadband compensation.
    private static final double WHEEL_MAXIMUM_POWER = 1.0;

    private DcMotorAccelerationThread wheelAccelerationThread;
    private DcMotorController wheelControllerLeft;
    private DcMotorController wheelControllerRight;
    private DcMotorAccelerated wheelLeftFront;
    private DcMotorAccelerated wheelLeftRear;
    private DcMotorAccelerated wheelRightFront;
    private DcMotorAccelerated wheelRightRear;

 /*   public Drivetrain(HardwareMap aHardwareMap) {
        wheelControllerLeft = aHardwareMap.dcMotorController.get("wheelsLeft");
        wheelControllerRight = aHardwareMap.dcMotorController.get("wheelsRight");
        wheelLeftFront = new DcMotorAccelerated(aHardwareMap.dcMotor.get(FRONTLEFT_WHEEL_NAME), WHEEL_ACCEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_DECEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_MINIMUM_POWER, WHEEL_MAXIMUM_POWER);
        wheelLeftRear = new DcMotorAccelerated(aHardwareMap.dcMotor.get(BACKLEFT_WHEEL_NAME), WHEEL_ACCEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_DECEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_MINIMUM_POWER, WHEEL_MAXIMUM_POWER);
        wheelRightFront = new DcMotorAccelerated(aHardwareMap.dcMotor.get(FRONTRIGHT_WHEEL_NAME), WHEEL_ACCEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_DECEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_MINIMUM_POWER, WHEEL_MAXIMUM_POWER);
        wheelRightRear = new DcMotorAccelerated(aHardwareMap.dcMotor.get(BACKRIGHT_WHEEL_NAME), WHEEL_ACCEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_DECEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_MINIMUM_POWER, WHEEL_MAXIMUM_POWER);

        wheelRightFront.setDirection(REVERSE);
        wheelRightRear.setDirection(REVERSE);
        runWithoutEncoderPid();

        wheelAccelerationThread = new DcMotorAccelerationThread();
        wheelAccelerationThread.addMotor(wheelLeftFront);
        wheelAccelerationThread.addMotor(wheelLeftRear);
        wheelAccelerationThread.addMotor(wheelRightFront);
        wheelAccelerationThread.addMotor(wheelRightRear);
        wheelAccelerationThread.start();
    }

    public void brake() {
        //Use this method for instantly stopping in the middle of an opmode.
        //DO NOT use this in the stop() method of an opmode.
        wheelLeftFront.stopMotorHard();
        wheelLeftRear.stopMotorHard();
        wheelRightFront.stopMotorHard();
        wheelRightRear.stopMotorHard();
    }

    public void runWithoutEncoderPid() {
        DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        //do something to replace all these with just the motors and the statement above
        wheelControllerLeft.setMotorMode(ENCODER_PORT_2, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelControllerRight.setMotorMode(ENCODER_PORT_1, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelControllerRight.setMotorMode(ENCODER_PORT_2, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double aLeftPower, double aRightPower, LinearOpMode opMode) {
        if((aLeftPower < 0) == (aRightPower < 0))
        {
            setAccelerationRate(WHEEL_ACCEL_SPEED_PER_SECOND_STRAIGHT, WHEEL_DECEL_SPEED_PER_SECOND_STRAIGHT);
        }
        else
        {
            setAccelerationRate(WHEEL_ACCEL_SPEED_PER_SECOND_TURNING, WHEEL_DECEL_SPEED_PER_SECOND_TURNING);
        }

        mechanumTeleOp(opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y,-opMode.gamepad1.right_stick_x);
        wheelLeftFront.setTargetPower(wheelSpeeds[3]);
        wheelLeftRear.setTargetPower(wheelSpeeds[0]);
        wheelRightFront.setTargetPower(wheelSpeeds[4]);
        wheelRightRear.setTargetPower(wheelSpeeds[1]);
    }

    public void setPowerWithoutAcceleration(double aLeftPower, double aRightPower) {
        wheelLeftFront.setDirectPower(aLeftPower);
        wheelLeftRear.setDirectPower(aLeftPower);
        wheelRightFront.setDirectPower(aRightPower);
        wheelRightRear.setDirectPower(aRightPower);
    }

    private void setAccelerationRate(double anAcceleration, double aDeceleration){
        wheelLeftFront.setAccelerationRates(anAcceleration, aDeceleration);
        wheelLeftRear.setAccelerationRates(anAcceleration, aDeceleration);
        wheelRightFront.setAccelerationRates(anAcceleration, aDeceleration);
        wheelRightRear.setAccelerationRates(anAcceleration, aDeceleration);
    }

    public void setMinimumMotorPower(double aMinimumPower) {
        wheelLeftFront.setMinPower(aMinimumPower);
        wheelLeftRear.setMinPower(aMinimumPower);
        wheelRightFront.setMinPower(aMinimumPower);
        wheelRightRear.setMinPower(aMinimumPower);
    }

    public void stop() {
        //Use this method for ending the thread and stopping the motors at the end of an opmode.
        //DO NOT use this in the middle of an opmode as it kills the thread.
        wheelAccelerationThread.stop();
    }



    double wheelSpeeds[] = new double[4];


    public void mechanumTeleOp(double x, double y, double rotation) {

        wheelSpeeds[0] = x + y + rotation;
        wheelSpeeds[1] = -x + y - rotation;
        wheelSpeeds[2] = -x + y + rotation;
        wheelSpeeds[3] = x + y - rotation;

        normalize(wheelSpeeds);

        //leftDriveBack.setPower(wheelSpeeds[0]);
        //rightDriveBack.setPower(wheelSpeeds[1]);
        //leftDriveFront.setPower(wheelSpeeds[2]);
        //rightDriveFront.setPower(wheelSpeeds[3]);
    }   //mecanumDrive_Cartesian

    private void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);

        for (int i = 1; i < wheelSpeeds.length; i++) {
            double magnitude = Math.abs(wheelSpeeds[i]);

            if (magnitude > maxMagnitude) {
                maxMagnitude = magnitude;
            }
        }

        if (maxMagnitude > 0.75) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }


    }

public void stopDrive(){
    wheelAccelerationThread.stop();
}
*/
}