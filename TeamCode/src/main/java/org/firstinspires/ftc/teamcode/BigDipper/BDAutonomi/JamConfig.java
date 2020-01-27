package org.firstinspires.ftc.teamcode.BigDipper.BDAutonomi;

import com.qualcomm.robotcore.util.ReadWriteFile;

public class JamConfig {
    public static JamSelector.JAlliance alliance = JamSelector.deserializeAlliance();
    public static JamSelector.JGoal goal = JamSelector.deserializeGoal();
    public static JamSelector.JDestination destination = JamSelector.deserializeDestination();
}
