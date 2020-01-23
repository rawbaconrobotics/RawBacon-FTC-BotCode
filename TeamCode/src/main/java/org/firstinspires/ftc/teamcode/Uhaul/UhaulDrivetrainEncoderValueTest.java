package org.firstinspires.ftc.teamcode.Uhaul;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.BaseLinearOpMode;


/**
 * @author Raw Bacon Coders
 * Tests the Uhaul Drivetrain Encoder
 */
@TeleOp(name="Uhaul Drivetrain Encoder Test", group="Uhaul")
@Disabled

public class UhaulDrivetrainEncoderValueTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Servo latchServo = null;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDriveBack = null;
    private DcMotor rightDriveBack = null;

    /**
     * Provides encoder counts through telemetry
     */
@Override
    public void runOpMode() throws InterruptedException {





        /**
         * Checks for wheel movement and outputs encoder counts to the screen
         * @throws InterruptedException
         */

            leftDrive  = hardwareMap.get(DcMotor.class, "left_drive_front");
            rightDrive = hardwareMap.get(DcMotor.class, "right_drive_front");
            leftDriveBack = hardwareMap.get(DcMotor.class, "left_drive_back");
            rightDriveBack = hardwareMap.get(DcMotor.class, "right_drive_back");

            // Most robots need the motor on one side to be reversed to drive forward
            // Reverse the motor that runs backwards when connected directly to the battery
            leftDrive.setDirection(DcMotor.Direction.FORWARD);
            leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
            rightDrive.setDirection(DcMotor.Direction.REVERSE);
            rightDriveBack.setDirection(DcMotor.Direction.REVERSE);
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            waitForStart();


            while(opModeIsActive()){
                telemetry.addData("Left Front",  " Position: %7d", leftDrive.getCurrentPosition());
                telemetry.addData("Right Front",  " Position: %7d", rightDrive.getCurrentPosition());
                telemetry.addData("Left Back",  " Position: %7d", leftDriveBack.getCurrentPosition());
                telemetry.addData("Right Back",  " Position: %7d", rightDriveBack.getCurrentPosition());
                telemetry.update();
            }
        }
    }


