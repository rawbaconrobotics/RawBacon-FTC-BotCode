package org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulComponentImplBase;

    public class UhaulArm extends UhaulComponentImplBase {

        private ElapsedTime runtime = new ElapsedTime();
        public Servo uhaulArm1 = null;
        public Servo uhaulArm2 = null;

        public UhaulArm(LinearOpMode opMode) {
            super(opMode);
        }

        @Override
        public void init() {
            uhaulArm1 = hardwareMap.servo.get("uhaul_arm2");
            uhaulArm2 = hardwareMap.servo.get("uhaul_arm2");
            uhaulArm1.setPosition(0.5);
            uhaulArm2.setPosition(0.5);
        }

        public void moveArm() {
            uhaulArm1.setPosition((gamepad2.left_stick_x + 1) / 2);
            uhaulArm2.setPosition(1-(gamepad2.left_stick_x + 1) / 2);
        }

        @Override
        public void initAutonomous() {

        }
    }