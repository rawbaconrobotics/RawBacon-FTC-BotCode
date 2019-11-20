package org.firstinspires.ftc.teamcode.BlockemSockem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Clyde_OLD.HardwareHelper;

public class Latch implements HardwareHelper {
    private final static double right_Latch_Down = 1;
    private final static double right_Latch_Up = 0;
    private final static double left_Latch_Down = 1;
    private final static double left_Latch_Up = 0;

    private Servo right_Latch;
    private Servo left_Latch;

    public void init(HardwareMap mappy) {
        right_Latch = mappy.get(Servo.class, BESE_HW_Names.RIGHTLATCH );
        left_Latch = mappy.get(Servo.class, BESE_HW_Names.LEFTLATCH);
        latch_Go_Up();
    }

    public void latch_Go_Down() {
        right_Latch.setPosition(right_Latch_Down);
        left_Latch.setPosition(left_Latch_Down);
    }
    public void latch_Go_Up() {
        right_Latch.setPosition(right_Latch_Up);
        left_Latch.setPosition(left_Latch_Up);
    }
}

