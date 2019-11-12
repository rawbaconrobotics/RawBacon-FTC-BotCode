 /*Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/


 package org.firstinspires.ftc.teamcode.Clyde_OLD.MaxHerrera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

 /**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 * */



@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
@Disabled
public class MHAutoCodingChallenge extends LinearOpMode {

 //Declare OpMode members.

    MHHardwarePushbot    robot   = new MHHardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime  runtime = new ElapsedTime();
    MHFunctionDungeon1   funhold = new MHFunctionDungeon1();
    ModernRoboticsI2cGyro   Spyro    = null;
    ColorSensor c_sensorRight = null;
    ColorSensor c_sensorLeft = null;

     static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

         // Initialize the drive system variables.
         // The init() method of the hardware class does all the work here


        robot.init(hardwareMap);


        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotor  lift;
        Servo    latch1;
        Servo    latch2;

        lift  = hardwareMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        latch1 = hardwareMap.get(Servo.class, "latch1");
        latch1.setPosition(0.5);
        latch2 = hardwareMap.get(Servo.class, "latch2");
        latch2.setPosition(0.5);

        c_sensorRight = hardwareMap.get(ColorSensor.class, "right_color_sensor");
        c_sensorLeft = hardwareMap.get(ColorSensor.class, "left_color_sensor");

        DigitalChannel LiftStop;
        LiftStop = hardwareMap.get(DigitalChannel.class, "liftStop2_bottom");
        LiftStop.setMode(DigitalChannel.Mode.INPUT);

        Spyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        Spyro.calibrate();
        while(Spyro.isCalibrating() && !isStopRequested()){
            sleep(50);
            idle();
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // 15 in = 90 degrees, for turning

        //Movements;
        //move forward until both color sensors see red or blue
        // reset gyro
        //save the tape color in InitTapeColor
        //move forward 45 Inches
        //turn CW if the var is blue (0), CCW if it is red (1) by 90 degrees
        //move forward until the left (if blue) or right (if red) color sensor reports red or blue
        //move, forward if the reported color doesn't match up with the start color, backward if they do match up, 15 Inches
        //turn the opposite direction done previously by 90 degrees
        //move forward 3 Inches
        //close both servo latches
        //spin the lift motor until the touch sensor is triggered (pulses true when triggered)             //Ten steps total

        //robot.latch1.setPosition(); //opens/closes the latch, 0=closed, 0.5=open
        //robot.latch2.setPosition(); //opens/closes the latch, 0=closed, 0.5=open
        //funhold.QuadMotorSetPow(left, right, 0, 0);
        //funhold.encoderDrive(0, DRIVE_SPEED, Inches, timeoutS);
        //******using treads with 2 motors and **NOT** 4 mecanum wheels******

        char InitTapeColor;

        while (c_sensorRight.argb() != Color.RED && c_sensorRight.argb() != Color.BLUE){      //Step One
            if (c_sensorRight.argb() != Color.RED && c_sensorRight.argb() != Color.BLUE){
                robot.leftDrive.setPower(0.6);
            }
            else {
                robot.leftDrive.setPower(0);
            }
            if (c_sensorLeft.argb() != Color.RED && c_sensorLeft.argb() != Color.BLUE){
                robot.rightDrive.setPower(0.6);
            }
            else {
                robot.rightDrive.setPower(0);
            }
        }

        Spyro.resetZAxisIntegrator();                   //Step 1.5

        InitTapeColor = CurTapeColor();                //Step Two      //red=R, blue=B


        StraightDrive(0.6, 0, 45);   //Step Three

        if(InitTapeColor == 'R'){                         //Step Four
            funhold.encoderDrive(2, TURN_SPEED, -15, 5.0);
        }
        else{
            funhold.encoderDrive(2, TURN_SPEED, 15, 5.0);
        }

        if (InitTapeColor == 'R'){                    //Step Five
            while(c_sensorRight.argb() != Color.RED && c_sensorRight.argb() != Color.BLUE){
                funhold.QuadMotorSetPow(0.6, 0.6, 0, 0);
            }

        }
        else{
            while(c_sensorLeft.argb() != Color.RED && c_sensorLeft.argb() != Color.BLUE){
                funhold.QuadMotorSetPow(0.6, 0.6, 0, 0);
            }
            funhold.QuadMotorSetPow(0, 0, 0, 0);
        }

        if(InitTapeColor == CurTapeColor()){                   //Step Six
            funhold.encoderDrive(0, DRIVE_SPEED, -15, 5.0);
        }
        else{
            funhold.encoderDrive(0, DRIVE_SPEED, 15, 5.0);
        }

        if(InitTapeColor == 'R'){                     //Step Seven
            funhold.encoderDrive(2, TURN_SPEED, 15, 5.0);
        }
        else{
            funhold.encoderDrive(2, TURN_SPEED, -15, 5.0);
        }

        funhold.encoderDrive(0, DRIVE_SPEED, 3, 2.0); //Step Eight

        latch1.setPosition(0);                       //Step Nine
        latch2.setPosition(0);

        while(!LiftStop.getState()){                  //Step Ten
            lift.setPower(1);
        }
        lift.setPower(0);

        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();

        //Fin
    }

    public void StraightDrive(double Speed, int DesiredAngle, double Distance){
        double TargetCounts = Distance*COUNTS_PER_INCH;
        double Steer;
        double Lspeed;
        double Rspeed;


        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() +  (int)TargetCounts);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + (int) TargetCounts);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftDrive.setPower(Speed);
        robot.rightDrive.setPower(Speed);

        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy()) {
            Steer = getSteer(DesiredAngle);
            if (Distance < 0){
                Steer *= -1;
            }

            Lspeed = Speed - Steer;
            Rspeed = Speed + Steer;

            if (Math.max(Math.abs(Lspeed), Math.abs(Rspeed)) > 1.0)
            {
                Lspeed /= Math.max(Math.abs(Lspeed), Math.abs(Rspeed));
                Rspeed /= Math.max(Math.abs(Lspeed), Math.abs(Rspeed));
            }

            robot.leftDrive.setPower(Lspeed);
            robot.rightDrive.setPower(Rspeed);
        }

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

public void AccuTurn(double DesiredAngle){
        while(getSteer(DesiredAngle) != 0){
            
        }
    }


    public double getSteer(double DesiredAngle){
        double error = DesiredAngle - Spyro.getIntegratedZValue();
        while(error > 180){error -= 360;}
        while(error <= -180){error += 360;}
        return Range.clip(error * 0.12, -1, 1);
    }

    public char CurTapeColor(){
        char TapeColor = 'N';
        if (c_sensorRight.argb() == Color.RED) {
            TapeColor = 'R';
        }
        else if(c_sensorRight.argb() == Color.BLUE){
            TapeColor = 'B';
        }
        else {
            if (c_sensorLeft.argb() == Color.RED) {
                TapeColor = 'R';
            }
            else if(c_sensorLeft.argb() == Color.BLUE){
                TapeColor = 'B';
            }
        }
        return TapeColor;
    }
}
