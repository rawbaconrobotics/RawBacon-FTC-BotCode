package org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents;

import com.qualcomm.ftccommon.FtcRobotControllerService;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
    double MAX_TICKS_BEFORE_OVERRIDE = 9115;
    double LIFT_MAX_SPEED = 1;
    double LIFT_SPEED_IN_AUTONOMOUS = 1;

    double kp = 1;
    double ki = 0;
    double kd = 0;
    double kf = 0;


    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime dpadtime = new ElapsedTime();

    public DcMotorEx uhaulLift = null;
    public DcMotorEx uhaulLiftTwo = null;

    private static final double COUNTS_PER_MOTOR_REV = 383.6;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 0.315;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public double dpadBlocks = 1;

    boolean liftCompensate = false;

    public static double leftPosition = 0;
    public static double rightPosition = 0;
    double leftTarget = 0;
    double rightTarget = 0;

    double dtPosition = 0;

    boolean override = false;




    public static double liftEncoderSetpoint = 0;
    public static boolean liftIsBusy = false;
    boolean comingUp = false;


    public enum LiftState {
        NOT_MOVING,
        MOVING,
        COMPENSATING

    }
    public static LiftState liftState = LiftState.NOT_MOVING;



    /**
     * Overrides the default opmode for UhaulLift
     */
    public UhaulLift(LinearOpMode opMode) {
        super(opMode);
    }

    /**
     * Initializes the proccess
     */
    @Override
    public void init() {
        dpadtime.reset();
        uhaulLift = (DcMotorEx) hardwareMap.dcMotor.get(UHAUL_LIFT_1);
        uhaulLiftTwo = (DcMotorEx) hardwareMap.dcMotor.get(UHAUL_LIFT_2);

        //    PIDFCoefficients pidNew = new PIDFCoefficients(kp, ki, kd, kf);
        //     uhaulLift.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        //     uhaulLiftTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        uhaulLift.setDirection(DcMotor.Direction.FORWARD);
        uhaulLiftTwo.setDirection(DcMotor.Direction.FORWARD);

        uhaulLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        uhaulLiftTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        uhaulLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        uhaulLiftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        uhaulLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uhaulLiftTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftPosition = (-uhaulLift.getCurrentPosition());
        rightPosition = (-uhaulLiftTwo.getCurrentPosition());

        leftTarget = Math.max(0, -uhaulLift.getTargetPosition());
        rightTarget = Math.max(0, -uhaulLiftTwo.getTargetPosition());




    }

    /**
     * Initializes the proccess for testing purposes
     */
    public void initForTesting() {
        uhaulLift = (DcMotorEx) hardwareMap.dcMotor.get(UHAUL_LIFT_1);
        uhaulLiftTwo = (DcMotorEx) hardwareMap.dcMotor.get(UHAUL_LIFT_2);
        uhaulLift.setDirection(DcMotor.Direction.FORWARD);
        uhaulLiftTwo.setDirection(DcMotor.Direction.FORWARD);
        uhaulLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        uhaulLiftTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        uhaulLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        uhaulLiftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftPosition = (-uhaulLift.getCurrentPosition());
        rightPosition = (-uhaulLiftTwo.getCurrentPosition());
        leftTarget = Math.max(0, -uhaulLift.getTargetPosition());
        rightTarget = Math.max(0, -uhaulLiftTwo.getTargetPosition());
    }

    /**
     * Initializes the proccess for the autonomous
     */
    @Override
    public void initAutonomous() {

        uhaulLift = (DcMotorEx) hardwareMap.dcMotor.get(UHAUL_LIFT_1);
        uhaulLiftTwo = (DcMotorEx) hardwareMap.dcMotor.get(UHAUL_LIFT_2);

        //    PIDFCoefficients pidNew = new PIDFCoefficients(kp, ki, kd, kf);
        //    uhaulLift.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        //    uhaulLiftTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        uhaulLift.setDirection(DcMotor.Direction.FORWARD);
        uhaulLiftTwo.setDirection(DcMotor.Direction.FORWARD);

        uhaulLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        uhaulLiftTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        uhaulLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        uhaulLiftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        uhaulLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uhaulLiftTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftPosition = (-uhaulLift.getCurrentPosition());
        rightPosition = (-uhaulLiftTwo.getCurrentPosition());

        leftTarget = Math.max(0, -uhaulLift.getTargetPosition());
        rightTarget = Math.max(0, -uhaulLiftTwo.getTargetPosition());


    }


//Would be nice to use state machines here and enums! Example:   https://gm0.copperforge.cc/en/latest/docs/software/fundamental-concepts.html#finite-state-machines-and-enums

    /**
     * Defines the lift for the teleop
     */
    public void liftTeleOp() {


        leftPosition = Math.max(0, (-uhaulLift.getCurrentPosition()));
        rightPosition = Math.max(0, (-uhaulLiftTwo.getCurrentPosition()));




        if ((gamepad2.dpad_up || gamepad2.dpad_down) && (Math.abs(gamepad2.right_stick_y) < 0.1)) {
            if (gamepad2.dpad_up && (dpadtime.seconds() > .2)) {
                dpadBlocks++;
                dpadBlocks = (Range.clip(dpadBlocks, 1, 10));
                dpadtime.reset();
                comingUp = true;


            } else if (gamepad2.dpad_down &&(dpadtime.seconds() > .2)) {
                dpadBlocks--;
                dpadBlocks = (Range.clip(dpadBlocks, 1, 10));
                dpadtime.reset();
                comingUp = false;

            }


            switch ((int) dpadBlocks) {

                case 1:
                    liftEncoderSetpoint = 0;
                    break;
                case 2:
                    liftEncoderSetpoint = 800;
                    break;
                case 3:
                    liftEncoderSetpoint = 923+40;
                    break;
                case 4:
                    liftEncoderSetpoint = 1480+40;
                    break;
                case 5:
                    liftEncoderSetpoint = 2220+40;
                    break;
                case 6:
                    liftEncoderSetpoint = 3200+40;
                    break;
                case 7:
                    liftEncoderSetpoint = 4580+40;
                    break;
                case 8:
                    liftEncoderSetpoint = 6160+40;
                    break;
                case 9:
                    liftEncoderSetpoint = 8400+40;
                    break;
                case 10:
                    liftEncoderSetpoint = 8050;
                    break;
            }


        }
//TODO Add these values
        //TODO Slow mode while lift is up!


        else if (gamepad2.a) {

liftState = LiftState.MOVING;

        }
        else if (gamepad2.right_stick_button) {

            liftEncoderSetpoint = 0;
            dpadBlocks = 0;
            liftState = LiftState.MOVING;

        }
        else if (gamepad2.dpad_right) {
            if(gamepad2.right_bumper){
                uhaulLiftTwo.setPower(0.25);
            }
            else if(gamepad2.left_bumper){
                uhaulLiftTwo.setPower(-0.25);
            }
            else{
                uhaulLiftTwo.setPower(0);
            }
        }

        else if (gamepad2.dpad_left) {
            if(gamepad2.right_bumper){
                uhaulLift.setPower(0.25);
            }
            else if(gamepad2.left_bumper){
                uhaulLift.setPower(-0.25);
            }
            else{
                uhaulLift.setPower(0);
            }
        }


        else if ((dpadBlocks == 1) && (liftState == LiftState.NOT_MOVING)) {
            if ((((leftPosition + rightPosition) / 2) < MAX_TICKS_BEFORE_OVERRIDE) && (Math.abs(gamepad2.right_stick_y) > 0.1)) {
                //   uhaulLift.setPower(gamepad2.right_stick_y / 2);
                //   uhaulLiftTwo.setPower(gamepad2.right_stick_y /2);

                uhaulLift.setPower(gamepad2.right_stick_y / 1.25);
                uhaulLiftTwo.setPower(gamepad2.right_stick_y / 1.25);

                System.out.println("SET POWER TO " + Double.toString(gamepad2.right_stick_y / 2));
            } else if ((((leftPosition + rightPosition) / 2) < MAX_TICKS_BEFORE_OVERRIDE) && (Math.abs(gamepad2.right_stick_y) <= 0.1)) {
                //do nothing, it's already at the right speed
                System.out.println("POSITION SET TO 0");
                uhaulLift.setPower(0);
                uhaulLiftTwo.setPower(0);
                liftIsBusy = false;

            } else {
                override = true;
            }


        } else if ((Math.abs(gamepad2.right_stick_y) > 0.1) && (liftState == LiftState.NOT_MOVING)) {
            if (((leftPosition + rightPosition) / 2) < MAX_TICKS_BEFORE_OVERRIDE) {
                uhaulLift.setPower(gamepad2.right_stick_y / 2.5);
                uhaulLiftTwo.setPower(gamepad2.right_stick_y / 2.5);
            } else {
                override = true;
            }

        } else if(!override && (liftState == LiftState.NOT_MOVING)) {
            uhaulLift.setPower(0);
            uhaulLiftTwo.setPower(0);

        }


        if (override) {

            liftEncoderSetpoint = (int) Math.abs((liftFunction(1 * BLOCK_HEIGHT) * COUNTS_PER_MOTOR_REV));

            override = false;

liftState = LiftState.MOVING;

        }
        if (gamepad2.b) {

        liftState = LiftState.COMPENSATING;

        }


        if (liftState == LiftState.MOVING) {
            teleOpEncoderDrive();
        }
        if (liftState == LiftState.COMPENSATING) {
            liftErrorCompensate();
        }

        telemetry.addData("Current Dpad Blocks Set To: ", (int) dpadBlocks);
        telemetry.addData("Lift", (int) leftPosition);
        telemetry.addData("Lift2", (int) rightPosition);
        telemetry.addData("the encoder ticks we want: ", (int) liftEncoderSetpoint);
        if (override) {
            telemetry.addData("OVERRIDE ", "TRUE!");
        } else {
            telemetry.addData("override", "false");
        }
        telemetry.update();


    }

    enum LIFT_DIRECTION{
        UP,
        DOWN,
        NONE
    }
    LIFT_DIRECTION direction = LIFT_DIRECTION.NONE;

    enum RUE_SETTER{
        NOTSET,
        SET,
    }
    RUE_SETTER ruestate = RUE_SETTER.NOTSET;

    public void teleOpEncoderDrive() {
        liftIsBusy = true;
        runtime.reset();


        //   if (liftEncoderSetpoint  < ((Math.max(0, (-uhaulLift.getCurrentPosition())) + (Math.max(0, (-uhaulLiftTwo.getCurrentPosition())))) / 2) && (liftState == LiftState.MOVING) && ((direction == LIFT_DIRECTION.DOWN) || (direction == LIFT_DIRECTION.NONE))) {
     //   if ((dpadBlocks != 2) || (dpadBlocks != 3)) {
            if (ruestate == RUE_SETTER.NOTSET) {

                uhaulLift.setTargetPosition(-(int) liftEncoderSetpoint);
                uhaulLiftTwo.setTargetPosition(-(int) liftEncoderSetpoint);

                uhaulLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                uhaulLiftTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (liftEncoderSetpoint < (-uhaulLift.getCurrentPosition())) {
                    uhaulLift.setPower(LIFT_SPEED_IN_AUTONOMOUS);
                    uhaulLiftTwo.setPower(LIFT_SPEED_IN_AUTONOMOUS);

                } else {
                    uhaulLift.setPower(-LIFT_SPEED_IN_AUTONOMOUS);
                    uhaulLiftTwo.setPower(-LIFT_SPEED_IN_AUTONOMOUS);
                }

                ruestate = RUE_SETTER.SET;

            }


            // if(-liftEncoderSetpoint < uhaulLift.getCurrentPosition() ){
            if (opModeIsActive() &&
                    (runtime.seconds() < 15) && (gamepad2.right_stick_y < 0.1) &&
                    ((uhaulLift.getCurrentPosition() < (uhaulLift.getTargetPosition() -50)) || (uhaulLift.getCurrentPosition() > (uhaulLift.getTargetPosition() +50)) || (uhaulLiftTwo.getCurrentPosition() < (uhaulLiftTwo.getTargetPosition() -50)) || uhaulLiftTwo.getCurrentPosition() > (uhaulLiftTwo.getTargetPosition() +50))
                    && !gamepad2.dpad_up && !gamepad2.dpad_down) {


                telemetry.addData("Current Dpad Blocks Set To: ", (int) dpadBlocks);
                telemetry.addData("Lift", (int) leftPosition);
                telemetry.addData("Lift2", (int) rightPosition);
                telemetry.addData("the encoder ticks we want: ", (int) liftEncoderSetpoint);
                if (override) {
                    telemetry.addData("OVERRIDE ", "TRUE!");
                } else {
                    telemetry.addData("override", "false");
                }
                telemetry.addData("CURRENTLY ACCEPTING NO INPUT,", "MOVING AUTOMATICALLY");
                telemetry.update();
            } else if (gamepad2.right_stick_y < 0.1) {

                uhaulLift.setPower(0);
                uhaulLiftTwo.setPower(0);

                uhaulLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                uhaulLiftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //liftState = LiftState.COMPENSATING;
                liftState = LiftState.NOT_MOVING;
                direction = LIFT_DIRECTION.NONE;

                ruestate = RUE_SETTER.NOTSET;

            } else {
                liftState = LiftState.NOT_MOVING;
                direction = LIFT_DIRECTION.NONE;

                uhaulLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                uhaulLiftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                ruestate = RUE_SETTER.NOTSET;
            }

        }
    /*    else if((dpadBlocks == 2) || (dpadBlocks == 3)){


            if(ruestate == RUE_SETTER.NOTSET){

                uhaulLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                uhaulLiftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                if(dpadBlocks == 2){
                    desiredTicks = uhaulLiftTwo.getCurrentPosition();
                    problem = 2;
                    uhaulLift.setPower(-1);
                }
                else if((dpadBlocks == 3)){
                    desiredTicks = uhaulLift.getCurrentPosition();
                    problem = 3;
                    uhaulLiftTwo.setPower(-1);
                }



                ruestate = RUE_SETTER.SET;

            }


            if(problem == 2){


                if (opModeIsActive() &&
                        (runtime.seconds() < 15) && (gamepad2.right_stick_y < 0.1) &&
                        ((-uhaulLift.getCurrentPosition()) < (600))
                        && !gamepad2.dpad_up && !gamepad2.dpad_down) {


                    telemetry.addData("Current Dpad Blocks Set To: ", (int) dpadBlocks);
                    telemetry.addData("Lift", (int) leftPosition);
                    telemetry.addData("Lift2", (int) rightPosition);
                    telemetry.addData("the encoder ticks we want: ", (int) liftEncoderSetpoint);
                    if (override) {
                        telemetry.addData("OVERRIDE ", "TRUE!");
                    } else {
                        telemetry.addData("override", "false");
                    }
                    telemetry.addData("CURRENTLY ACCEPTING NO INPUT,", "MOVING AUTOMATICALLY");
                    telemetry.update();
                }else if(gamepad2.right_stick_y < 0.1) {

                    uhaulLift.setPower(0);
                    uhaulLiftTwo.setPower(0);



                    //liftState = LiftState.COMPENSATING;
                    liftState = LiftState.NOT_MOVING;
                    direction = LIFT_DIRECTION.NONE;

                    ruestate = RUE_SETTER.NOTSET;

                }

                else{
                    liftState = LiftState.NOT_MOVING;
                    direction = LIFT_DIRECTION.NONE;


                    ruestate = RUE_SETTER.NOTSET;
                }



            }else if(problem == 3){

                if (opModeIsActive() &&
                        (runtime.seconds() < 15) && (gamepad2.right_stick_y < 0.1) &&
                        ((-uhaulLift.getCurrentPosition()) < (963))
                        && !gamepad2.dpad_up && !gamepad2.dpad_down ) {


                    telemetry.addData("Current Dpad Blocks Set To: ", (int) dpadBlocks);
                    telemetry.addData("Lift", (int) leftPosition);
                    telemetry.addData("Lift2", (int) rightPosition);
                    telemetry.addData("the encoder ticks we want: ", (int) liftEncoderSetpoint);
                    if (override) {
                        telemetry.addData("OVERRIDE ", "TRUE!");
                    } else {
                        telemetry.addData("override", "false");
                    }
                    telemetry.addData("CURRENTLY ACCEPTING NO INPUT,", "MOVING AUTOMATICALLY");
                    telemetry.update();
                }else if(gamepad2.right_stick_y < 0.1) {

                    uhaulLift.setPower(0);
                    uhaulLiftTwo.setPower(0);



                    //liftState = LiftState.COMPENSATING;
                    liftState = LiftState.NOT_MOVING;
                    direction = LIFT_DIRECTION.NONE;

                    ruestate = RUE_SETTER.NOTSET;

                }

                else{
                    liftState = LiftState.NOT_MOVING;
                    direction = LIFT_DIRECTION.NONE;

                    ruestate = RUE_SETTER.NOTSET;
                }




            }



        }
        }
*/




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
//return a set number of rotations (ticks wanted /383.6) depending on what dpadblock value is eg. if dpadblocks = 1 return 0;
        return rotations_from_zero;

    }

    double ACCEPTABLE_ERROR_TICKS = 20;

    double desiredTicks;
    int problem;
    public void liftErrorCompensate()

    {



            liftIsBusy = true;
            runtime.reset();


            //   if (liftEncoderSetpoint  < ((Math.max(0, (-uhaulLift.getCurrentPosition())) + (Math.max(0, (-uhaulLiftTwo.getCurrentPosition())))) / 2) && (liftState == LiftState.MOVING) && ((direction == LIFT_DIRECTION.DOWN) || (direction == LIFT_DIRECTION.NONE))) {

            if(ruestate == RUE_SETTER.NOTSET){

                uhaulLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                uhaulLiftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                if((-uhaulLift.getCurrentPosition()) > (-uhaulLiftTwo.getCurrentPosition())){
                    desiredTicks = uhaulLiftTwo.getCurrentPosition();
                    problem = 1;
                    uhaulLift.setPower(0.2);
                }
                else if((-uhaulLift.getCurrentPosition()) < -uhaulLiftTwo.getCurrentPosition()){
                    desiredTicks = uhaulLift.getCurrentPosition();
                    problem = 2;
                    uhaulLiftTwo.setPower(0.2);
                }



                ruestate = RUE_SETTER.SET;

            }


            if(problem == 1){


                if (opModeIsActive() &&
                        (runtime.seconds() < 15) && (gamepad2.right_stick_y < 0.1) &&
                        ((-uhaulLift.getCurrentPosition()) > (-desiredTicks))
                        && !gamepad2.dpad_up && !gamepad2.dpad_down && (Math.abs(((-uhaulLift.getCurrentPosition()) - (-uhaulLiftTwo.getCurrentPosition()))) < 300)) {


                    telemetry.addData("Current Dpad Blocks Set To: ", (int) dpadBlocks);
                    telemetry.addData("Lift", (int) leftPosition);
                    telemetry.addData("Lift2", (int) rightPosition);
                    telemetry.addData("the encoder ticks we want: ", (int) liftEncoderSetpoint);
                    if (override) {
                        telemetry.addData("OVERRIDE ", "TRUE!");
                    } else {
                        telemetry.addData("override", "false");
                    }
                    telemetry.addData("CURRENTLY ACCEPTING NO INPUT,", "MOVING AUTOMATICALLY");
                    telemetry.update();
                }else if(gamepad2.right_stick_y < 0.1) {

                    uhaulLift.setPower(0);
                    uhaulLiftTwo.setPower(0);



                    //liftState = LiftState.COMPENSATING;
                    liftState = LiftState.NOT_MOVING;
                    direction = LIFT_DIRECTION.NONE;

                    ruestate = RUE_SETTER.NOTSET;

                }

                else{
                    liftState = LiftState.NOT_MOVING;
                    direction = LIFT_DIRECTION.NONE;


                    ruestate = RUE_SETTER.NOTSET;
                }



            }else if(problem == 2){

                if (opModeIsActive() &&
                        (runtime.seconds() < 15) && (gamepad2.right_stick_y < 0.1) &&
                        ((-uhaulLiftTwo.getCurrentPosition()) > (-desiredTicks))
                        && !gamepad2.dpad_up && !gamepad2.dpad_down && (Math.abs(((-uhaulLift.getCurrentPosition()) - (-uhaulLiftTwo.getCurrentPosition()))) < 300)) {


                    telemetry.addData("Current Dpad Blocks Set To: ", (int) dpadBlocks);
                    telemetry.addData("Lift", (int) leftPosition);
                    telemetry.addData("Lift2", (int) rightPosition);
                    telemetry.addData("the encoder ticks we want: ", (int) liftEncoderSetpoint);
                    if (override) {
                        telemetry.addData("OVERRIDE ", "TRUE!");
                    } else {
                        telemetry.addData("override", "false");
                    }
                    telemetry.addData("CURRENTLY ACCEPTING NO INPUT,", "MOVING AUTOMATICALLY");
                    telemetry.update();
                }else if(gamepad2.right_stick_y < 0.1) {

                    uhaulLift.setPower(0);
                    uhaulLiftTwo.setPower(0);



                    //liftState = LiftState.COMPENSATING;
                    liftState = LiftState.NOT_MOVING;
                    direction = LIFT_DIRECTION.NONE;

                    ruestate = RUE_SETTER.NOTSET;

                }

                else{
                    liftState = LiftState.NOT_MOVING;
                    direction = LIFT_DIRECTION.NONE;

                    ruestate = RUE_SETTER.NOTSET;
                }




            }


            // if(-liftEncoderSetpoint < uhaulLift.getCurrentPosition() ){

        }




















//TODO ADD MANUAL OPTIONAL ENCODER TICKS!

    public void liftTo(int block) {

        liftEncoderSetpoint = (int) Math.abs((((liftFunction(BLOCK_HEIGHT * (Range.clip(block, 1, 10))))) * COUNTS_PER_MOTOR_REV));


        runtime.reset();

        if (liftEncoderSetpoint < ((leftPosition + rightPosition)/2)) {



            uhaulLift.setPower(-LIFT_SPEED_IN_AUTONOMOUS);
            uhaulLiftTwo.setPower(-LIFT_SPEED_IN_AUTONOMOUS);



            while (opModeIsActive() &&
                    (runtime.seconds() < 15) &&
                    (((leftPosition + rightPosition)/2) > (liftEncoderSetpoint)) && (((leftPosition + rightPosition)/2) < MAX_TICKS_BEFORE_OVERRIDE)) {

                telemetry.addData("Lift", " Position: %7d", (int) leftPosition);
                telemetry.addData("Lift2", " Position: %7d", (int) rightPosition);
                telemetry.addData("the encoder ticks we want: ", (int) liftEncoderSetpoint);

                telemetry.update();
            }


                uhaulLift.setPower(0);
                uhaulLiftTwo.setPower(0);




        } else if (liftEncoderSetpoint > ((leftPosition + rightPosition)/2)) {
            runtime.reset();




            uhaulLift.setPower(LIFT_SPEED_IN_AUTONOMOUS);
            uhaulLiftTwo.setPower(LIFT_SPEED_IN_AUTONOMOUS);



            while (opModeIsActive() &&
                    (runtime.seconds() < 15) && (gamepad2.right_stick_y < 0.1) &&
                    (((leftPosition + rightPosition)/2) < (liftEncoderSetpoint)) && (((leftPosition + rightPosition)/2) < MAX_TICKS_BEFORE_OVERRIDE)) {

                telemetry.addData("Lift", " Position: %7d", (int) leftPosition);
                telemetry.addData("Lift2", " Position: %7d", (int) rightPosition);
                telemetry.addData("the encoder ticks we want: ", (int)liftEncoderSetpoint);

                telemetry.update();

            }

                uhaulLift.setPower(0);
                uhaulLiftTwo.setPower(0);




        }



        if(((leftPosition + ACCEPTABLE_ERROR_TICKS) < rightPosition) || ((rightPosition + ACCEPTABLE_ERROR_TICKS) < leftPosition)){

            double lowerPos = Math.min(leftPosition, rightPosition);

            if ((leftPosition != lowerPos)){

                uhaulLift.setPower(-.3);

                while (opModeIsActive() &&
                        (runtime.seconds() < 15) &&
                        (leftPosition > (lowerPos))){

                    telemetry.addData("Uhaul Lift 1", "Adjusting");
                    telemetry.addData("target position", (int) lowerPos);
                    telemetry.addData("current Position:", (int)leftPosition);
                    telemetry.update();

                }
                uhaulLift.setPower(0);
            } else if ((rightPosition != lowerPos)) {

                uhaulLiftTwo.setPower(-.3);

                while (opModeIsActive() &&
                        (runtime.seconds() < 15) &&
                        (rightPosition > (lowerPos))) {

                    telemetry.addData("Uhaul Lift 2", "Adjusting");
                    telemetry.addData("target position", (int)lowerPos);
                    telemetry.addData("current Position:", (int)rightPosition);
                    telemetry.update();


                }
                uhaulLiftTwo.setPower(0);

            }
        }


    }







//TODO Add these values
    //TODO Slow mode while lift is up

//block 2 = 560 ticks
    //block 3 = 923
    //block 4 = 1480
    //block 5 = 2220
    //block 6 = 3200
    //block 7 = 4580
    //block 8 = 6160
    //block 9 = 8400

    //max = 9115
    //capstone = 8050
































}

