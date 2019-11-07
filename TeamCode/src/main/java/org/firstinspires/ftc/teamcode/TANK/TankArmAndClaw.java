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

package org.firstinspires.ftc.teamcode.TANK;

import com.qualcomm.hardware.HardwareDeviceManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TANK.TankComponentImplBase;
public class TankArmAndClaw extends TankComponentImplBase{
    public void initAutonomous(){}


    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor tankArm = null;
    //private Servo tankClaw = null;
    private Servo tankLatch1 = null;
    private Servo tankLatch2 = null;
    public boolean latchPosition = true;
    double latch1Open  = 0.9;
    double latch1Closed = 0.2;

    public TankArmAndClaw(LinearOpMode opMode)
    {
        super(opMode);
    }


    @Override

    public void init(){

        //tankArm = hardwareMap.dcMotor.get("tank+_arm");
        //tankClaw = hardwareMap.servo.get("tank_claw");
        tankLatch1 = hardwareMap.servo.get("tank_latch_1");
        tankLatch2 = hardwareMap.servo.get("tank_latch_2");


    }


    /*public void moveArm (){

        double armSpeed = 1;

        if (gamepad2.dpad_down){
            armSpeed = 0.5;
        }

        if (gamepad2.dpad_up){
            armSpeed = 1;
        }

        tankArm.setPower(gamepad2.right_stick_y*armSpeed);
    }


    /*public void moveClaw(){

        tankClaw.setPosition(gamepad2.left_stick_y/2 + 0.5);

    }
    */

    public void moveLatch (){

        if (gamepad2.a && latchPosition) {
            tankLatch1.setPosition(latch1Closed);
            tankLatch2.setPosition(latch1Open);
            latchPosition = false;
        }
        else if (gamepad2.a && !latchPosition){
                tankLatch1.setPosition(latch1Open);
                tankLatch2.setPosition(latch1Closed);
                
                latchPosition = true;
            }
        if (gamepad2.x) {
            tankLatch1.setPosition(latch1Closed);
            tankLatch2.setPosition(latch1Open);

        }
        if(gamepad2.y){
            tankLatch1.setPosition(latch1Open);
            tankLatch2.setPosition(latch1Closed);
            
        }


    }
    public void autoLatch(double position1, double position2){
        tankLatch1.setPosition(position1);
        tankLatch2.setPosition(position2);
    }
}
