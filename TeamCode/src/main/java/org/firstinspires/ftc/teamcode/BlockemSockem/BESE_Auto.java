package org.firstinspires.ftc.teamcode.BlockemSockem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class BESE_Auto extends LinearOpMode {
    Robot bese = new Robot();
    @Override
    public void runOpMode() {
        bese.init(hardwareMap);
        bese.resetEncoders();
        waitForStart();
    }
}
