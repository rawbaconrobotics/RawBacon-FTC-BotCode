package org.firstinspires.ftc.teamcode.Uhaul;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulComponentImplBase;


    public class UhaulLatch extends UhaulComponentImplBase  {

        private ElapsedTime runtime = new ElapsedTime();
        public Servo uhaulLatch = null;
        private static final double LATCH_OPEN_POSITION = 0.8;
        private static final double LATCH_CLOSED_POSITION = 0.2;

        public UhaulLatch(LinearOpMode opMode) {
            super(opMode);
        }

        @Override
        public void init() {
            uhaulLatch = hardwareMap.servo.get("uhaul_latch");
            uhaulLatch.setPosition(LATCH_OPEN_POSITION);
        }

        public void moveLatch () {
            if (gamepad1.right_bumper || gamepad1.left_bumper) {

                uhaulLatch.setPosition(LATCH_OPEN_POSITION);
            }
            if (gamepad1.right_trigger == 1 || gamepad1.left_trigger == 1) {

                uhaulLatch.setPosition(LATCH_CLOSED_POSITION);
            }
        }
    }