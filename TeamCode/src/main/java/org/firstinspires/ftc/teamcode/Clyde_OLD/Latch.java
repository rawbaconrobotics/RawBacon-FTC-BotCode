package org.firstinspires.ftc.teamcode.Clyde_OLD;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Latch implements HardwareHelper {
    private Servo hook;
    private final double UP_POSITION = 0.0;
    private final double DOWN_POSITION = 1.0;

    public void init(HardwareMap mappy){
        hook = mappy.get(Servo.class, ClydeHWNames.LATCH);
        up();
    }

    public void up(){
        hook.setPosition(UP_POSITION);
    }
    public void down(){
        hook.setPosition(DOWN_POSITION);
    }
}
