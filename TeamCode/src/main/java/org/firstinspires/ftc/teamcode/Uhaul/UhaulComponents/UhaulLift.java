package org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Uhaul.UhaulComponents.UhaulComponentImplBase;

    public class UhaulLift extends UhaulComponentImplBase {

        private ElapsedTime runtime = new ElapsedTime();
        public DcMotor uhaulLift = null;

        public UhaulLift(LinearOpMode opMode) {
            super(opMode);
        }

        @Override
        public void init() {
            uhaulLift = hardwareMap.dcMotor.get("uhaul_lift");
            uhaulLift.setDirection(DcMotor.Direction.FORWARD);

            }

        public void moveLift () {
            uhaulLift.setPower(gamepad2.right_stick_y);

        }
        @Override
        public void initAutonomous() {

        }
    }