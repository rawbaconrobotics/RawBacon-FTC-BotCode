package org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.RobotComponentImplBase;
import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulComponentImplBase;

import static android.os.SystemClock.sleep;

/**
 * Defines the Uhaul Lift
 * @author Raw Bacon Coders
 */
public class UhaulLift extends UhaulComponentImplBase {

    String UHAUL_LIFT_1 = "uhaul_lift_1";
    String UHAUL_LIFT_2 = "uhaul_lift_2";
    double BLOCK_HEIGHT = 5;
    double INITIAL_HEIGHT = 0; //height to get to before the first block
    double MAX_TICKS_BEFORE_OVERRIDE = (liftFunction(40))*(COUNTS_PER_MOTOR_REV);
    double LIFT_MAX_SPEED = 1;
    double LIFT_SPEED_IN_AUTONOMOUS = 0.5;

    double kp = 1;
    double ki = 0;
    double kd = 0;
    double kf = 0;


    private ElapsedTime runtime = new ElapsedTime();

    public DcMotorEx uhaulLift = null;
    public DcMotorEx uhaulLiftTwo = null;

    private static final double COUNTS_PER_MOTOR_REV = 383.6;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 0.315;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double COUNTS_PER_DEGREE = 15;





    public int dpadBlocks = 0;


    /** Overrides the default opmode for UhaulLift */
    public UhaulLift(LinearOpMode opMode) {
        super(opMode);
    }

    /** Initializes the proccess */
    @Override
    public void init() {
        uhaulLift = (DcMotorEx) hardwareMap.dcMotor.get(UHAUL_LIFT_1);
        uhaulLiftTwo = (DcMotorEx) hardwareMap.dcMotor.get(UHAUL_LIFT_2);

     //   PIDFCoefficients pidNew = new PIDFCoefficients(kp, ki, kd, kf);
     //   uhaulLift.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidNew);
     //   uhaulLiftTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidNew);

        uhaulLift.setDirection(DcMotor.Direction.FORWARD);
        uhaulLiftTwo.setDirection(DcMotor.Direction.FORWARD);

       uhaulLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        uhaulLiftTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        uhaulLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        uhaulLiftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        uhaulLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uhaulLiftTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    /** Initializes the proccess for testing purposes */
public void initForTesting(){
    //we don't want it to brake on 0 power!
    uhaulLift = (DcMotorEx) hardwareMap.dcMotor.get(UHAUL_LIFT_1);
    uhaulLiftTwo = (DcMotorEx) hardwareMap.dcMotor.get(UHAUL_LIFT_2);

   // PIDFCoefficients pidNew = new PIDFCoefficients(kp, ki, kd, kf);
   // uhaulLift.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
   // uhaulLiftTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

    uhaulLift.setDirection(DcMotor.Direction.FORWARD);
    uhaulLiftTwo.setDirection(DcMotor.Direction.FORWARD);
    uhaulLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    uhaulLiftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


}
//increment encoder positions using specified dpad presses in order to go up that many blocks high
    //have joystick at .25 if a dpad value is above 0 and then at .5 when dpad = 0
    //take away driver control after encoder value reached-ish
    //display current dpad value in telemetry

    //try to find a way to check if the button goes up or down during the loop

    boolean override = false;
    int liftEncoderSetpoint = 0;

//Would be nice to use state machines here and enums! Example:   https://gm0.copperforge.cc/en/latest/docs/software/fundamental-concepts.html#finite-state-machines-and-enums
    boolean comingUp = false;
    /** Defines the lift for the teleop */
    public void liftTeleOp() {

        if ((gamepad2.dpad_up || gamepad2.dpad_down) && (Math.abs(gamepad2.right_stick_y) < 0.1)) {
            if (gamepad2.dpad_up) {
                dpadBlocks++;
                comingUp = true;
                sleep(500);

            } else {
                if(dpadBlocks > 0) {
                    dpadBlocks--;
                }
                else{
                    dpadBlocks = 0;
                }
                comingUp = false;
                sleep(500);
            }

            if((dpadBlocks == 1) && comingUp){
                liftEncoderSetpoint = (int) ((INITIAL_HEIGHT + (liftFunction(BLOCK_HEIGHT * dpadBlocks))) * COUNTS_PER_MOTOR_REV);

            }
            else{
                liftEncoderSetpoint = (int) (((liftFunction(BLOCK_HEIGHT * dpadBlocks))) * COUNTS_PER_MOTOR_REV);

            }

            teleOpEncoderDrive();
           // telemetry.addData("TELEOP,", "ENCODER DRIVE");
           // telemetry.update();



        } else if (gamepad2.a) {
            dpadBlocks = 0;

            liftEncoderSetpoint = 0;

            teleOpEncoderDrive();


        } else if (dpadBlocks == 0) { //TODO Change back to 0
            if ((-uhaulLift.getCurrentPosition() > -MAX_TICKS_BEFORE_OVERRIDE) && (Math.abs(gamepad2.right_stick_y) > 0.1)) {
                uhaulLift.setPower(gamepad2.right_stick_y / 2);
                uhaulLiftTwo.setPower(gamepad2.right_stick_y /2);
            } else if((-uhaulLift.getCurrentPosition() > -MAX_TICKS_BEFORE_OVERRIDE) && (Math.abs(gamepad2.right_stick_y) <= 0.1)) {
                //do nothing, it's already at the right speed
                uhaulLift.setPower(0);
                uhaulLiftTwo.setPower(0);
            }
            else{
                    override = true;
                }



        } else if (override) {

            liftEncoderSetpoint = (int) (liftFunction(5.1) * COUNTS_PER_MOTOR_REV);

            override = false;

            teleOpEncoderDrive();



        } else if(gamepad2.right_stick_y > 0.1) {
            uhaulLift.setPower(gamepad2.right_stick_y / 4);
            uhaulLiftTwo.setPower(gamepad2.right_stick_y / 4);

        }else if(gamepad2.b){
            liftErrorCompensate();
        }
        else if(gamepad2.left_stick_button || gamepad2.right_stick_button){
            while (gamepad2.left_stick_button) {
                uhaulLiftTwo.setPower(gamepad2.right_stick_y / 4);
            }
            while (gamepad2.right_stick_button) {
                uhaulLift.setPower(gamepad2.right_stick_y / 4);
            }
        }
        else{
            uhaulLift.setPower(0);
            uhaulLiftTwo.setPower(0);

        }

       telemetry.addData("Current Dpad Blocks Set To: ", dpadBlocks);
        telemetry.addData("Press 'A' on gamepad 2", " to reset!");
        telemetry.addData("Lift", " Position: %7d", -uhaulLift.getCurrentPosition());
        telemetry.addData("Lift2", " Position: %7d", -uhaulLiftTwo.getCurrentPosition());
        telemetry.addData("Current Dpad Blocks Set To: ", dpadBlocks);
        telemetry.addData("Press 'A' on gamepad 2", " to reset!");
        telemetry.addData("the encoder ticks we want: ", liftEncoderSetpoint);
        telemetry.update();
        telemetry.update();


    }




    int previousBlocks = 0;

    /** Defines the liftAuto proccess */
    public void liftAuto(int howManyBlocks, double speed, double timeoutS) {


    }



    public void teleOpEncoderDrive() {
        runtime.reset();

        if (-liftEncoderSetpoint < -uhaulLift.getCurrentPosition()) {

            uhaulLift.setTargetPosition(-liftEncoderSetpoint);
            uhaulLiftTwo.setTargetPosition(-liftEncoderSetpoint);

            uhaulLift.setPower(-LIFT_MAX_SPEED);
            uhaulLiftTwo.setPower(-LIFT_MAX_SPEED);

            uhaulLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            uhaulLiftTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            while ((gamepad2.right_stick_y < 0.1) && opModeIsActive() &&
              //       (runtime.seconds() < 15) && (uhaulLiftTwo.isBusy())) {
                (runtime.seconds() < 15) && (uhaulLift.isBusy() && uhaulLiftTwo.isBusy())) {

                telemetry.addData("Lift", " Position: %7d", -uhaulLift.getCurrentPosition());
                telemetry.addData("Lift2", " Position: %7d", -uhaulLiftTwo.getCurrentPosition());

                telemetry.addData("Current Dpad Blocks Set To: ", dpadBlocks);
                telemetry.addData("Press 'A' on gamepad 2", " to reset!");
                telemetry.addData("the encoder ticks we want: ", liftEncoderSetpoint);
                telemetry.update();
            }

            if(gamepad2.right_stick_y < 0.1) {
                uhaulLift.setPower(0);
                uhaulLiftTwo.setPower(0);
                liftErrorCompensate();
            }


            uhaulLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            uhaulLiftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        } else if (-liftEncoderSetpoint > uhaulLift.getCurrentPosition()) {
        } else if (-liftEncoderSetpoint > -uhaulLift.getCurrentPosition()) {
            runtime.reset();


                uhaulLift.setTargetPosition(-liftEncoderSetpoint);
                uhaulLiftTwo.setTargetPosition(-liftEncoderSetpoint);

                uhaulLift.setPower(LIFT_MAX_SPEED);
                uhaulLiftTwo.setPower(LIFT_MAX_SPEED);

                uhaulLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               uhaulLiftTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                while ((gamepad2.right_stick_y < 0.1) && opModeIsActive() &&
                  //    (runtime.seconds() < 15) && (uhaulLiftTwo.isBusy())) {
                    (runtime.seconds() < 15) && (uhaulLift.isBusy() && uhaulLiftTwo.isBusy())) {

                    telemetry.addData("Lift", " Position: %7d", -uhaulLift.getCurrentPosition());
                    telemetry.addData("Lift2", " Position: %7d", -uhaulLiftTwo.getCurrentPosition());
                    telemetry.addData("Current Dpad Blocks Set To: ", dpadBlocks);
                    telemetry.addData("Press 'A' on gamepad 2", " to reset!");
                    telemetry.addData("the encoder ticks we want: ", liftEncoderSetpoint);
                    telemetry.update();
                }

            if(gamepad2.right_stick_y < 0.1) {
                uhaulLift.setPower(0);
                uhaulLiftTwo.setPower(0);
                liftErrorCompensate();

            }


                uhaulLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                uhaulLiftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }





    /** Initializes the proccess for the autonomous */
    @Override
    public void initAutonomous() {

        uhaulLift = (DcMotorEx) hardwareMap.dcMotor.get(UHAUL_LIFT_1);
        uhaulLiftTwo = (DcMotorEx) hardwareMap.dcMotor.get(UHAUL_LIFT_2);

        PIDFCoefficients pidNew = new PIDFCoefficients(kp, ki, kd, kf);
        uhaulLift.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidNew);
        uhaulLiftTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidNew);

        uhaulLift.setDirection(DcMotor.Direction.FORWARD);
        uhaulLiftTwo.setDirection(DcMotor.Direction.FORWARD);

        uhaulLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        uhaulLiftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        uhaulLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uhaulLiftTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


    /** Defines the targets */
    public void liftFor(double blocks, double speed, double timeoutS) {
        liftEncoderSetpoint = (int) ((INITIAL_HEIGHT + (liftFunction(BLOCK_HEIGHT * blocks))) * COUNTS_PER_MOTOR_REV);

        runtime.reset();

        if (-liftEncoderSetpoint < -uhaulLift.getCurrentPosition()) {

            uhaulLift.setTargetPosition(-liftEncoderSetpoint);
            uhaulLiftTwo.setTargetPosition(-liftEncoderSetpoint);

            uhaulLift.setPower(-speed);
            uhaulLiftTwo.setPower(-speed);

            uhaulLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            uhaulLiftTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            while ((gamepad2.right_stick_y < 0.1) && opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && (uhaulLift.isBusy() && uhaulLiftTwo.isBusy())) {

                telemetry.addData("Lift", " Position: %7d", -uhaulLift.getCurrentPosition());
                telemetry.addData("Current Dpad Blocks Set To: ", dpadBlocks);
                telemetry.addData("Press 'A' on gamepad 2", " to reset!");
                telemetry.addData("the encoder ticks we want: ", liftEncoderSetpoint);
                telemetry.update();
            }

            if(gamepad2.right_stick_y < 0.1) {
                uhaulLift.setPower(0);
                uhaulLiftTwo.setPower(0);
                liftErrorCompensate();
            }


            uhaulLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            uhaulLiftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        } else if (-liftEncoderSetpoint > uhaulLift.getCurrentPosition()) {
            runtime.reset();


            uhaulLift.setTargetPosition(-liftEncoderSetpoint);
            uhaulLiftTwo.setTargetPosition(-liftEncoderSetpoint);

            uhaulLift.setPower(speed);
            uhaulLiftTwo.setPower(speed);

            uhaulLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            uhaulLiftTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            while ((gamepad2.right_stick_y < 0.1) && opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && (uhaulLift.isBusy() && uhaulLiftTwo.isBusy())) {

                telemetry.addData("Lift", " Position: %7d", uhaulLift.getCurrentPosition());
                telemetry.addData("Current Dpad Blocks Set To: ", dpadBlocks);
                telemetry.addData("Press 'A' on gamepad 2", " to reset!");
                telemetry.addData("the encoder ticks we want: ", liftEncoderSetpoint);
                telemetry.update();
            }

            if(gamepad2.right_stick_y < 0.1) {
                uhaulLift.setPower(0);
                uhaulLiftTwo.setPower(0);
                liftErrorCompensate();

            }


            uhaulLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            uhaulLiftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


    }

    public double maxLeft, maxRight, max, ratio;

    public double[] normalizeAuto(double wheelspeeds0, double wheelspeeds1, double wheelspeeds2, double wheelspeeds3){
        /*
         * Are any of the computed wheel powers greater than 1?
         */

        if(Math.abs(wheelspeeds0) > 1
                || Math.abs(wheelspeeds1) > 1
                || Math.abs(wheelspeeds2) > 1
                || Math.abs(wheelspeeds3) > 1)
        {

            // Yeah, figure out which one

            maxLeft = Math.max(Math.abs(wheelspeeds0), Math.abs(wheelspeeds2));
            maxRight = Math.max(Math.abs(wheelspeeds1), Math.abs(wheelspeeds3));
            max = Math.max(maxLeft, maxRight);
            ratio = 1 / max; //Create a ratio to normalize them all
            double[] normalSpeeds = {(wheelspeeds0 * ratio), (wheelspeeds1 * ratio), (wheelspeeds2 * ratio), (wheelspeeds3 * ratio)};
            //leftDriveBack.setPower(wheelspeeds0 * ratio);
            //rightDriveBack.setPower(wheelspeeds1 * ratio);
            //leftDriveFront.setPower(wheelspeeds2 * ratio);
            //rightDriveFront.setPower(wheelspeeds3 * ratio);
            return normalSpeeds;
        }
        /*
         * Nothing we need to do to the raw powers
         */

        else
        {
            // leftDriveBack.setPower(wheelspeeds0);
            //  rightDriveBack.setPower(wheelspeeds1);
            // leftDriveFront.setPower(wheelspeeds2);
            // rightDriveFront.setPower(wheelspeeds3);
            double[] normalSpeedz = {(wheelspeeds0), (wheelspeeds1), (wheelspeeds2), (wheelspeeds3)};
            return normalSpeedz;
        }
    }
    double rotations_from_zero = 0;
    public double liftFunction(double inches_desired){
        rotations_from_zero = ((8587-Math.sqrt(74863129-(44803*(Math.pow(inches_desired, 2)))))  / 200);

        return rotations_from_zero;

    }

    public void liftErrorCompensate()

    {
        if ((-uhaulLift.getCurrentPosition() < -uhaulLiftTwo.getCurrentPosition() + 10) || (-uhaulLift.getCurrentPosition() > -uhaulLiftTwo.getCurrentPosition() - 10)) {
           // if (1==10) {

            int lowerPos = Math.min(-uhaulLift.getCurrentPosition(), -uhaulLiftTwo.getCurrentPosition());

            if ((-uhaulLift.getCurrentPosition() != lowerPos) && (gamepad2.right_stick_y < 0.1)) {
                uhaulLift.setTargetPosition(-lowerPos);
                uhaulLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                uhaulLift.setPower(-.3);
                while (uhaulLift.isBusy()) {
                    telemetry.addData("Uhaul Lift 1", "Adjusting");
                    telemetry.addData("target position", -lowerPos);
                    telemetry.addData("current Position:", -uhaulLift.getCurrentPosition());
                    telemetry.update();
                }
                uhaulLift.setPower(0);
            } else if ((-uhaulLiftTwo.getCurrentPosition() != lowerPos) && (gamepad2.right_stick_y < 0.1)) {
                uhaulLiftTwo.setTargetPosition(-lowerPos);
                uhaulLiftTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                uhaulLiftTwo.setPower(-.3);
                while (uhaulLiftTwo.isBusy()) {
                    telemetry.addData("Uhaul Lift 2", "Adjusting");
                    telemetry.addData("target position", -lowerPos);
                    telemetry.addData("current Position:", -uhaulLiftTwo.getCurrentPosition());
                    telemetry.update();
                }
                uhaulLiftTwo.setPower(0);

            }
        }
    }

}

