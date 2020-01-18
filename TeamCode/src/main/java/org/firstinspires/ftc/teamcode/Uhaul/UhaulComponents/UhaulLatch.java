package org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulComponentImplBase;


    public class UhaulLatch extends UhaulComponentImplBase  {
//READY TO GO FOR UHAUL
        private ElapsedTime runtime = new ElapsedTime();

        public Servo uhaulLatch = null;
        public Servo uhaulLatchTwo = null;

        private static final double LATCH_OPEN_POSITION = 0;
        private static final double LATCH_CLOSED_POSITION = 0.3;

        private final static String LATCH_SERVO_1 = "uhaul_latch_1" ;
        private final static String LATCH_SERVO_2 = "uhaul_latch_2" ;

        public UhaulLatch(LinearOpMode opMode) {
            super(opMode);
        }

        @Override
        public void init() {
            uhaulLatch = hardwareMap.servo.get(LATCH_SERVO_1);
            uhaulLatchTwo = hardwareMap.servo.get(LATCH_SERVO_2);
        }

        public void moveLatch () {
            if (gamepad1.right_bumper || gamepad1.left_bumper) {

                uhaulLatch.setPosition(LATCH_OPEN_POSITION);
            }
            if (gamepad1.right_trigger >= 0.5 || gamepad1.left_trigger >= 0.5) {

                uhaulLatch.setPosition(LATCH_CLOSED_POSITION);
            }
        }
        @Override
        public void initAutonomous() {
            uhaulLatch = hardwareMap.servo.get(LATCH_SERVO_1);
            uhaulLatchTwo = hardwareMap.servo.get(LATCH_SERVO_2);
            uhaulLatch.setPosition(LATCH_OPEN_POSITION);
            uhaulLatchTwo.setPosition(LATCH_OPEN_POSITION);
        }
        public void openLatch(){
            System.out.println("LATCH OPENING");
            uhaulLatch.setPosition(LATCH_OPEN_POSITION);
            uhaulLatchTwo.setPosition(LATCH_OPEN_POSITION);
        }

        public void closeLatch(){
            System.out.println("LATCH CLOSING");
            uhaulLatch.setPosition(LATCH_CLOSED_POSITION);
            uhaulLatchTwo.setPosition(LATCH_CLOSED_POSITION);
        }
    }