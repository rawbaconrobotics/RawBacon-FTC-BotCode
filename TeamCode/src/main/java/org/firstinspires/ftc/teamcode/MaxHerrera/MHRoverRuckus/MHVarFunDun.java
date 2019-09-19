/* Copyright (c) 2017 FIRST. All rights reserved.
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
 */

package org.firstinspires.ftc.teamcode.MaxHerrera.MHRoverRuckus;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
//@Disabled
public class MHVarFunDun extends LinearOpMode {      //I have NO clue what this means, but everything else is fixed, error-wise

    MHVarHardware         robot   = new MHVarHardware();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 / 40.0;     // This is < 1.0 if geared UP
    static final double     LIFT_GEAR_REDUCTION     = 1 / 60.0;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     LIFT_COUNTS_PER_INCH    = (COUNTS_PER_MOTOR_REV * LIFT_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    
    public void encoderDrive(int Type, double speed, double Inches, double timeoutS) {

        int leftTarget;
        int rightTarget;
        int left2Target;
        int right2Target;
        int[][] Types = { {1, 1, 1, 1}, //+ Inches goes forward  /  - Inches goes backward
                          {-1, 1, 1, -1}, //+ Inches goes left  /  - Inches goes right
                          {1, -1, 1, -1}, //+ Inches goes CW  /  - Inches goes CCW, 15 in = 90 degrees
                          {1, 0, 0, 1}, //+ Inches goes forwards-right  /  - Inches goes backwards-left
                          {0, 1, 1, 0} //+ Inches goes forwards-left  /  - Inches goes backwards-right
                        };

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            leftTarget = robot.leftDrive.getCurrentPosition() + (int)(Types[Type][0] * Inches * COUNTS_PER_INCH);
            rightTarget = robot.rightDrive.getCurrentPosition() + (int)(Types[Type][1] * Inches * COUNTS_PER_INCH);
            left2Target = robot.left2Drive.getCurrentPosition() + (int)(Types[Type][2] * Inches * COUNTS_PER_INCH);
            right2Target = robot.right2Drive.getCurrentPosition() + (int)(Types[Type][3] * Inches * COUNTS_PER_INCH);

            // Determine new target position, and pass to motor controller

            robot.leftDrive.setTargetPosition(leftTarget);
            robot.rightDrive.setTargetPosition(rightTarget);
            robot.left2Drive.setTargetPosition(left2Target);
            robot.right2Drive.setTargetPosition(right2Target);

            //robot.leftDrive.setTargetPosition();
            //robot.rightDrive.setTargetPosition();
            //robot.left2Drive.setTargetPosition();
            //robot.right2Drive.setTargetPosition();

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left2Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right2Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            QuadMotorSetPow(Math.abs(speed), Math.abs(speed), Math.abs(speed), Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    ((robot.leftDrive.isBusy() || Types[Type][0] == 0) && (robot.rightDrive.isBusy() || Types[Type][1] == 0) &&
                            (robot.left2Drive.isBusy() || Types[Type][2] == 0) && (robot.right2Drive.isBusy() || Types[Type][3] == 0))) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", leftTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d", robot.leftDrive.getCurrentPosition(), robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            QuadMotorSetPow(0,0,0,0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.left2Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right2Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    /*public void alignToTapeyCraterate() {

        while (robot.color_sensor != Color.RED || Color.BLUE){
            QuadMotorSetPow(1, 1, 0, 0);
        }

        if (robot.color_sensor == Color.RED || Color.BLUE){
            encoderDrive(0,DRIVE_SPEED,5,2.0);
        }

    }*/

    public void LiftMeFromTheGround(int Direction){ //1 to extend lift, -1 to retract lift
        robot.Lift.setTargetPosition(robot.Lift.getCurrentPosition() + (int) (Direction*4.5*LIFT_COUNTS_PER_INCH));
        robot.Lift.setPower(1);
        while (opModeIsActive() && robot.Lift.isBusy()) {}
        robot.Lift.setPower(0);
    }


    public void QuadMotorSetPow(double left, double right, double left2, double right2){
        robot.leftDrive.setPower(left);
        robot.rightDrive.setPower(right);
        robot.left2Drive.setPower(left2);
        robot.right2Drive.setPower(right2);
    }

}