package org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BigDipper.RobotComponents.RobotComponentImplBase;
import org.firstinspires.ftc.teamcode.Uhaul.AutonomousSelector;
import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulComponentImplBase;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.Uhaul.AutonomousSelector.deserializeAlliance;

/**
 * Defines the Uhaul Lift
 * @author Raw Bacon Coders
 */
public class UhaulLift extends UhaulComponentImplBase {
//DONE FOR UHAUL!

    String UHAUL_LIFT_1 = "uhaul_lift_1";
    String UHAUL_LIFT_2 = "uhaul_lift_2";
    double BLOCK_HEIGHT = 4;
    double INITIAL_HEIGHT = 5; //height to get to before the first block
    double MAX_TICKS_BEFORE_OVERRIDE = 80000;
    double LIFT_MAX_SPEED = 0.5;
    double LIFT_SPEED_IN_AUTONOMOUS = 0.5;
    double kp = 0.3;
    double ki = 0.1;
    double kd = 0;
    double kf = 12.6;


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

        //PIDFCoefficients pidNew = new PIDFCoefficients(kp, ki, kd, kf);
        //uhaulLift.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        //uhaulLiftTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        uhaulLift.setDirection(DcMotor.Direction.FORWARD);
        uhaulLiftTwo.setDirection(DcMotor.Direction.FORWARD);
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

    /** Defines the lift for the teleop */
    public void liftTeleOp() {

        if((uhaulLift.getTargetPosition() == uhaulLift.getCurrentPosition() || ((gamepad2.right_stick_y != 0) && !override)) && (uhaulLift.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)){
            uhaulLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            uhaulLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else if(uhaulLift.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            uhaulLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            uhaulLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if ((gamepad2.dpad_up || gamepad2.dpad_down) && (gamepad2.right_stick_y == 0)) {
            if (gamepad2.dpad_up) {
                dpadBlocks++;
            } else {
                dpadBlocks--;
            }

            liftEncoderSetpoint = (int) ((BLOCK_HEIGHT * dpadBlocks) * COUNTS_PER_INCH);
            uhaulLift.setTargetPosition(liftEncoderSetpoint);
            uhaulLiftTwo.setTargetPosition(liftEncoderSetpoint);
            uhaulLift.setPower(LIFT_MAX_SPEED);
            uhaulLiftTwo.setPower(LIFT_MAX_SPEED);



        } else if (gamepad2.a) {
            dpadBlocks = 0;


        } else if (dpadBlocks == 0) {
            if ((uhaulLift.getCurrentPosition() < MAX_TICKS_BEFORE_OVERRIDE) && (gamepad2.right_stick_y != 0)) {
                uhaulLift.setPower(gamepad2.right_stick_y / 2);
                uhaulLiftTwo.setPower(gamepad2.right_stick_y /2);
            } else if((uhaulLift.getCurrentPosition() < MAX_TICKS_BEFORE_OVERRIDE) && (gamepad2.right_stick_y == 0)) {
                //do nothing, it's already at the right speed
            }
            else{
                    override = true;
                }



        } else if (override) {

            liftEncoderSetpoint = (int) ((-2) * COUNTS_PER_INCH);
            uhaulLift.setTargetPosition(liftEncoderSetpoint);
            uhaulLiftTwo.setTargetPosition(liftEncoderSetpoint);
            uhaulLift.setPower(LIFT_MAX_SPEED);
            uhaulLiftTwo.setPower(LIFT_MAX_SPEED);

            override = false;


        } else {
            uhaulLift.setPower(gamepad2.right_stick_y / 4);

        }

        telemetry.addData("Current Dpad Blocks Set To: ", dpadBlocks);
        telemetry.addData("Press 'A' on gamepad 2", " to reset!");

    }

    int previousBlocks = 0;

    /** Defines the liftAuto proccess */
    public void liftAuto(int howManyBlocks) {

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        uhaulLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        uhaulLiftTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        uhaulLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        uhaulLiftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if ((howManyBlocks < previousBlocks) && (howManyBlocks == 0)) {
            encoderDrive((-previousBlocks * BLOCK_HEIGHT) - INITIAL_HEIGHT);
        } else if ((howManyBlocks > previousBlocks) && (previousBlocks == 0)) {
            encoderDrive((howManyBlocks * BLOCK_HEIGHT) + INITIAL_HEIGHT);
        }
        //above are the normal situations, but what if we want to go from one block to another?
        else if (howManyBlocks != previousBlocks) {
            encoderDrive((howManyBlocks - previousBlocks) * BLOCK_HEIGHT);
        } else {
            //the current and previous blocks are the same, do nothing
        }
        previousBlocks = howManyBlocks;
    }


    /** Initializes the proccess for the autonomous */
    @Override
    public void initAutonomous() {

        uhaulLift = (DcMotorEx) hardwareMap.dcMotor.get(UHAUL_LIFT_1);
        uhaulLiftTwo = (DcMotorEx) hardwareMap.dcMotor.get(UHAUL_LIFT_2);



        uhaulLift.setDirection(DcMotor.Direction.FORWARD);
        uhaulLiftTwo.setDirection(DcMotor.Direction.FORWARD);
        uhaulLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        uhaulLiftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    /** Defines the targets */
    public void encoderDrive(double heightInInches) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = uhaulLift.getCurrentPosition() + (int)(heightInInches * COUNTS_PER_INCH);
            newRightTarget = uhaulLiftTwo.getCurrentPosition() + (int)(heightInInches * COUNTS_PER_INCH);
            uhaulLift.setTargetPosition(newLeftTarget);
            uhaulLiftTwo.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            uhaulLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            uhaulLiftTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            if(newRightTarget < uhaulLift.getCurrentPosition()){
                uhaulLift.setPower(-LIFT_SPEED_IN_AUTONOMOUS);
                uhaulLiftTwo.setPower(-LIFT_SPEED_IN_AUTONOMOUS);
            }
            else{
                uhaulLift.setPower(LIFT_SPEED_IN_AUTONOMOUS);
                uhaulLiftTwo.setPower(LIFT_SPEED_IN_AUTONOMOUS);

            }


            while (opModeIsActive() &&
                    (runtime.seconds() < 10) &&
                    (uhaulLift.isBusy() || uhaulLiftTwo.isBusy())) {

                if(!uhaulLift.isBusy()){
                    uhaulLift.setPower(0);
                }
                if(!uhaulLiftTwo.isBusy()){
                    uhaulLiftTwo.setPower(0);
                }

            }

            // Stop all motion;
            uhaulLift.setPower(0);
            uhaulLiftTwo.setPower(0);

            // Turn off RUN_TO_POSITION
            uhaulLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            uhaulLiftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }
}

