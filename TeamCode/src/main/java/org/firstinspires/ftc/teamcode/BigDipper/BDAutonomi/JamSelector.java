package org.firstinspires.ftc.teamcode.BigDipper.BDAutonomi;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@TeleOp(name = "JamSelector")
@Disabled
public class JamSelector extends LinearOpMode {
    JGoal goal = JGoal.PARK;
    JDestination destination = JDestination.MIDDLE;
    JAlliance alliance = JAlliance.RED;

    public static final String allianceFileName = "JAllianceConfig.json";
    public static final String goalFileName = "JGoalConfig.json";
    public static final String destinationFileName = "JDestinationConfig.json";

    public enum JGoal {
        FOUNDATION,
        STONES,
        PARK
    }

    public enum JDestination {
        WALL,
        MIDDLE
    }

    public enum JAlliance {
        RED,
        BLUE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        int stepNumber = 0;

        waitForStart();

        File allianceFile = AppUtil.getInstance().getSettingsFile(allianceFileName);
        File goalFile = AppUtil.getInstance().getSettingsFile(goalFileName);
        File destinationFile = AppUtil.getInstance().getSettingsFile(allianceFileName);

        while (opModeIsActive() && stepNumber < 3) {
            if (gamepad1.a) {
                switch (stepNumber) {
                    case 0: goal = JGoal.FOUNDATION; break;
                    case 1: destination = JDestination.MIDDLE; break;
                }
                stepNumber++;
            }
            if (gamepad1.b) {
                switch (stepNumber) {
                    case 0: goal = JGoal.STONES; break;
                    case 1: destination = JDestination.WALL; break;
                    case 2: alliance = JAlliance.RED; break;
                }
                stepNumber++;
            }
            if (gamepad1.x) {
                switch (stepNumber) {
                    case 0: goal = JGoal.PARK; break;
                    case 2: alliance = JAlliance.BLUE; break;
                }
                stepNumber++;
            }
        }

        ReadWriteFile.writeFile(allianceFile, serializeConfig(alliance));
        ReadWriteFile.writeFile(goalFile, serializeConfig(goal));
        ReadWriteFile.writeFile(destinationFile, serializeConfig(destination));
    }

    private String serializeConfig(Object config){
        return SimpleGson.getInstance().toJson(config);
    }

    public static JAlliance deserializeAlliance(){
        File allianceFile = AppUtil.getInstance().getSettingsFile(allianceFileName);
        String data = ReadWriteFile.readFile(allianceFile);
        return SimpleGson.getInstance().fromJson(data, JAlliance.class);
    }

    public static JGoal deserializeGoal(){
        File goalFile = AppUtil.getInstance().getSettingsFile(goalFileName);
        String data = ReadWriteFile.readFile(goalFile);
        return SimpleGson.getInstance().fromJson(data, JGoal.class);
    }

    public static JDestination deserializeDestination(){
        File destinationFile = AppUtil.getInstance().getSettingsFile(destinationFileName);
        String data = ReadWriteFile.readFile(destinationFile);
        return SimpleGson.getInstance().fromJson(data, JDestination.class);
    }
}
