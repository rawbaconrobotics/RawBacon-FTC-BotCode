package org.firstinspires.ftc.teamcode.TANK_OLD;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="TANKTeleOp_OLD", group="TANK_OLD")
public class TANKTeleOp_OLD extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public Servo TankLatch;

    TANK_OLD tank = new TANK_OLD(hardwareMap);

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        tank.drive.hwMap();
        tank.drive.setDirection();
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double leftPower;
        double rightPower;
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        /*/claw stuff
        if (gamepad2.a){
            tank.claw.TANKMoveClaw();
        }
        */
        /*if (gamepad2.b){
            tank.latch.moveLatch();
        }

         */
        if (gamepad1.left_bumper && !gamepad1.right_bumper){
            tank.drive.strafeLeftDrive(1);
        }else if (gamepad1.right_bumper){
            tank.drive.strafeRightDrive(1);
        }else{
            tank.drive.teleOpDrive();
        }
    }

    @Override
    public void stop() {
        tank.drive.halt();
    }

}