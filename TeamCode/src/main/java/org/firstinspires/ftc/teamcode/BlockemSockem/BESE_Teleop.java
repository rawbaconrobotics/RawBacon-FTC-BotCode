package org.firstinspires.ftc.teamcode.BlockemSockem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class BESE_Teleop extends OpMode {
    Robot bese = new Robot();

    @Override
    public void init() {
        bese.init(hardwareMap);
    }

    @Override
    public void loop() {
        bese.w.left_drive(-gamepad1.left_stick_x);
        bese.w.right_drive(-gamepad1.right_stick_x);

        if(gamepad1.right_bumper){
            bese.w.strafe(-0.5);
        }

        if(gamepad1.left_bumper){
            bese.w.strafe(0.5);
        }
    }
}
