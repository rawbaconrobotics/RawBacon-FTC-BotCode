package org.firstinspires.ftc.teamcode.Uhaul;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulComponentImplBase;


    public class UhaulDriveTrain extends UhaulComponentImplBase {

        private static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // eg: Neverest Motor Encoder
        private static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
        private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        private ElapsedTime runtime = new ElapsedTime();
        public DcMotor uhaulLeftBack = null;
        public DcMotor uhaulRightBack = null;
        public DcMotor uhaulLeftFront = null;
        public DcMotor uhaulRightFront = null;

        public UhaulDriveTrain(LinearOpMode opMode) {
            super(opMode);
        }

        public void init(){
            uhaulLeftBack = hardwareMap.dcMotor.get("left_drive_back");
            uhaulRightBack = hardwareMap.dcMotor.get("right_drive_back");
            uhaulLeftFront = hardwareMap.dcMotor.get("left_drive_front");
            uhaulRightFront = hardwareMap.dcMotor.get("right_drive_front");

            uhaulLeftBack.setDirection(DcMotor.Direction.REVERSE);
            uhaulRightBack.setDirection(DcMotor.Direction.FORWARD);
            uhaulRightFront.setDirection(DcMotor.Direction.FORWARD);
            uhaulLeftFront.setDirection(DcMotor.Direction.REVERSE);
        }

        public void wheelsTeleOp(){
            mechanumTeleOp(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
        }

        public void mechanumTeleOp(double x, double y, double rotation){
            double wheelSpeeds[] = new double[4];
            wheelSpeeds[0] = x + y + rotation;
            wheelSpeeds[1] = -x + y - rotation;
            wheelSpeeds[2] = -x + y + rotation;
            wheelSpeeds[3] = x + y - rotation;

            uhaulLeftFront.setPower(wheelSpeeds[2]);
            uhaulRightFront.setPower(wheelSpeeds[1]);
            uhaulRightBack.setPower(wheelSpeeds[0]);
            uhaulLeftBack.setPower(wheelSpeeds[3]);
        }






}
