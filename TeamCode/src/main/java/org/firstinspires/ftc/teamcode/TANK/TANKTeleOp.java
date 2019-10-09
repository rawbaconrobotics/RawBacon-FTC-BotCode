package org.firstinspires.ftc.teamcode.TANK;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="TANKTeleOp", group="TeleOp")
public class TANKTeleOp extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;

    TANK tank = new TANK(hardwareMap);

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
        tank.drive.teleOpDrive();
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

        /*//claw functions
        tank.claw.TANKMoveArm
        if (gamepad2.a) {
            tank.claw.TANKMoveClaw();
        }
        if (gamepad2.left_bumper) {
            tank.claw.TANKChangeArmSpeed();
        }*/
        //intake functions
        if (gamepad2.a){
            tank.intake.TANKIntake(true);
        }else{
            tank.intake.TANKIntake(false);
        }

    }

    @Override
    public void stop(){
        tank.shutdown();
    }
}