package org.firstinspires.ftc.teamcode.Uhaul.Autonomous;

/**
 * @author Raw Bacon Coders
 * This class holds the autonomous config options.
 */
public class AutoConfig {



    public AutonomousSelector.Alliance redAlliance = AutonomousSelector.deserializeAlliance();
    public AutonomousSelector.Options tasks = AutonomousSelector.deserializeOptions();
    public AutonomousSelector.Park park = AutonomousSelector.deserializePark();


}
