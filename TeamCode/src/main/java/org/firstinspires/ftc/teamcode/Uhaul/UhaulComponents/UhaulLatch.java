package org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulComponentImplBase;

/**
 * @author Raw Bacon Coders
 * Defines the UhaulLatch
 */
@Config
    public class UhaulLatch extends UhaulComponentImplBase  {
//READY TO GO FOR UHAUL
        private ElapsedTime runtime = new ElapsedTime();

        public Servo uhaulLatch = null;
        public Servo uhaulLatchTwo = null;

    public static double LATCH_OPEN_POSITION = .15;
        public static double LATCH_CLOSED_POSITION = 0.5;

        private final static String LATCH_SERVO_1 = "uhaul_latch_1" ;
        private final static String LATCH_SERVO_2 = "uhaul_latch_2" ;

        public UhaulLatch(LinearOpMode opMode) {
            super(opMode);
        }

        
         /** Initializes the uhaul latch */
        @Override
        public void init() {
            uhaulLatch = hardwareMap.servo.get(LATCH_SERVO_1);
            uhaulLatchTwo = hardwareMap.servo.get(LATCH_SERVO_2);
        }

         /** Defines how to move the latch using the gamepad. */
        public void moveLatch () {
            if (gamepad1.a) {

                uhaulLatch.setPosition(0.85);
                uhaulLatchTwo.setPosition(.15);
            }
            if (gamepad1.b) {

                uhaulLatch.setPosition(.15);
                uhaulLatchTwo.setPosition(.85);
            }
        }
         /** Initializes the autonomous */
        @Override
        public void initAutonomous() {
            uhaulLatch = hardwareMap.servo.get(LATCH_SERVO_1);
            uhaulLatchTwo = hardwareMap.servo.get(LATCH_SERVO_2);
            uhaulLatch.setPosition(0.15);
            uhaulLatchTwo.setPosition(.85);
        }
         /** Opens the latch */
        public void closeLatch(){
            System.out.println("LATCH OPENING");
            uhaulLatch.setPosition(0.85);
            uhaulLatchTwo.setPosition(.15);
        }
      /** Closes the latch */
        public void openLatch(){
            System.out.println("LATCH CLOSING");
            uhaulLatch.setPosition(.15);
            uhaulLatchTwo.setPosition(.85);
        }
    }
