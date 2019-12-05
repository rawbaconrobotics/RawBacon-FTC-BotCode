package org.firstinspires.ftc.teamcode.Uhaul;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulComponentImplBase;


public class UhaulIntake extends UhaulComponentImplBase {

    private static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // eg: Neverest Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor uhaulLeftIntake = null;
    public DcMotor uhaulRightIntake = null;


    public UhaulIntake(LinearOpMode opMode) {
        super(opMode);
    }

    public void init(){
        uhaulLeftIntake = hardwareMap.dcMotor.get("left_intake");
        uhaulRightIntake = hardwareMap.dcMotor.get("right_intake");

        uhaulLeftIntake.setDirection(DcMotor.Direction.REVERSE);
        uhaulRightIntake.setDirection(DcMotor.Direction.FORWARD);
    }
    public void runIntake(){
        while (gamepad2.right_trigger > 0){
            uhaulLeftIntake.setPower(1);
            uhaulRightIntake.setPower(1);
        }
        uhaulLeftIntake.setPower(0);
        uhaulRightIntake.setPower(0);
    }








}
