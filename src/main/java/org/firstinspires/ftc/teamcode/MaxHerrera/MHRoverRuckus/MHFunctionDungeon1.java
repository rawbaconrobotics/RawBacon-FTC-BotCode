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

package org.firstinspires.ftc.teamcode.Clyde_OLD.MHRoverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
@Disabled
public class MHFunctionDungeon1 extends LinearOpMode {

    MHHardwarePushbot         robot   = new MHHardwarePushbot();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
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

    public void encoderDrive(double Type, double speed, double Inches, double timeoutS) {

        int newTarget;
        int negnewTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            newTarget = robot.leftDrive.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);
            negnewTarget = robot.leftDrive.getCurrentPosition() + (int)(-Inches * COUNTS_PER_INCH);

            // Determine new target position, and pass to motor controller
            if (Type==0){ //forwards and backwards
                robot.leftDrive.setTargetPosition(newTarget);
                robot.rightDrive.setTargetPosition(newTarget);
            }

            if (Type==1){ //left and right (Strafing)
                robot.frontDrive.setTargetPosition(newTarget);
                robot.backDrive.setTargetPosition(newTarget);
            }

            if (Type==2){ //turning CW and CCW, 15 in=90 degrees
                robot.leftDrive.setTargetPosition(newTarget);
                robot.rightDrive.setTargetPosition(negnewTarget);
                robot.frontDrive.setTargetPosition(newTarget);
                robot.backDrive.setTargetPosition(negnewTarget);
            }
            
            if (Type==3) { //positive is forwards-right, negative is backwards-left
                robot.leftDrive.setTargetPosition(newTarget);
                robot.rightDrive.setTargetPosition(newTarget);
                robot.frontDrive.setTargetPosition(newTarget);
                robot.backDrive.setTargetPosition(newTarget);
            }

            if (Type==4) { //positive is forwards-left, negative is backwards-right
                robot.leftDrive.setTargetPosition(newTarget);
                robot.rightDrive.setTargetPosition(newTarget);
                robot.frontDrive.setTargetPosition(negnewTarget);
                robot.backDrive.setTargetPosition(negnewTarget);
            }

            //robot.leftDrive.setTargetPosition();
            //robot.rightDrive.setTargetPosition();
            //robot.frontDrive.setTargetPosition();
            //robot.backDrive.setTargetPosition();

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d", robot.leftDrive.getCurrentPosition(), robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            QuadMotorSetPow(0,0,0,0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    /*public void alignToTapeyCraterate() {

        while (color_sensor != color.red  && color.blue){
            QuadMotorSetPow(1, 1, 0, 0)
        }

        if (color_sensor == color.red || color.blue){
            encoderDrive(0,DRIVE_SPEED,5,2.0);
        }

    }*/

    public void LiftMeFromTheGround(double Direction){
        /*
        if (Direction == -1){
            while (!LiftStop1_bottom.isPressed || !LiftStop2_bottom.isPressed) {

                if (LiftStop1_bottom.isPressed){
                    robot.Lift1.setPower(-1);
                }
                else {
                    robot.Lift1.setPower(0);
                }

                if(LiftStop2_bottom.isPressed){
                    robot.Lift2.setPower(-1);
                }
                else {
                    robot.Lift2.setPower(0);
                }

            }
        }

        if (Direction == 1){
            while (!LiftStop1_top.isPressed || !LiftStop2_top.getState) {

                if (LiftStop1_top.isPressed){
                    robot.Lift1.setPower(1);
                }
                else {
                    robot.Lift1.setPower(0);
                }

                if(LiftStop2_top.isPressed){
                    robot.Lift2.setPower(1);
                }
                else {
                    robot.Lift2.setPower(0);
                }

            }
        } */
    }


    public void QuadMotorSetPow(double left, double right, double front, double back){
        robot.leftDrive.setPower(left);
        robot.rightDrive.setPower(right);
        robot.frontDrive.setPower(front);
        robot.backDrive.setPower(back);
    }

}