package org.firstinspires.ftc.teamcode.Uhaul;


import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ReadWriteFile;


/** 
 * @author Raw Bacon Coders
 * Selects which autonomous to use
 */
@TeleOp(name = "Autonomous Selector")
//@Disabled
public class AutonomousSelector extends LinearOpMode {

    /*
    How to use in autonomous:

            AutonomousSelector.deserializeOptions();
        if((optionsConfig.tasks.option()) == "AHP"){
        }
            AutonomousSelector.deserializeAlliance();
        if((AllianceConfig.redAlliance.option()) == true){
}





     */

    private AllianceConfig allianceConfig;
    private OptionsConfig optionsConfig;

    Telemetry.Item currentQuery;

    public static String allianceFileName = "AllianceConfig.json";
    public static String optionsFileName = "OptionsConfig.json";



    Func<String> message = new Func<String>() {
        @Override
        public String value(){
            return queries[stepNumber];
        }
    };


    /** 
     * Enum to represent options
     */
    public enum Options {
        DO_FOUNDATION,
        DO_STONE,
        DO_BOTH,
        DO_NEITHER,
        PARK_MIDDLE,
        PARK_WALL;

        public String option(){
            switch (this){
                case DO_FOUNDATION: return "DO FOUNDATION";
                case DO_STONE: return "DO STONE";
                case DO_BOTH: return "DO BOTH";
                case DO_NEITHER: return "DO NEITHER";
                case PARK_MIDDLE: return "PARK MIDDLE";
                case PARK_WALL: return "PARK WALL";
                default: return "NO CONFIG SET!";
            }
        }
    }
    /**
     * Enum to represent the alliances
     */
    public enum Alliance {
        RED,
        BLUE;

        public boolean redAlliancePressed(){
            switch (this){
                case RED: return true;
                case BLUE: return false;
                default: return false;
            }
        }
    }

    /**
     * Defines gamepad inputs
     */
    public enum Trigger {
        LEFT_STICK_Y,
        LEFT_STICK_X,
        RIGHT_STICK_Y,
        RIGHT_STICK_X,
        LEFT_TRIGGER,
        RIGHT_TRIGGER;

        public float getValue(Gamepad gamepad){
            switch (this){
                case LEFT_STICK_X: return gamepad.left_stick_x;
                case LEFT_STICK_Y: return gamepad.left_stick_y;
                case RIGHT_STICK_X: return gamepad.right_stick_x;
                case RIGHT_STICK_Y: return gamepad.right_stick_y;
                case LEFT_TRIGGER: return gamepad.left_trigger;
                case RIGHT_TRIGGER: return gamepad.right_trigger;
                default: return 0;
            }
        }
    }


    private int stepNumber = 0;


    /**
     * Runs the opmode
     */
    @Override public void runOpMode()throws InterruptedException {

        telemetry.log().add("-- Autonomous Selector --");
        telemetry.log().add("Press play to begin.");
        telemetry.log().add("");
        telemetry.log().add("REMEMBER: This opMode only selects your options,");
        telemetry.log().add("you still need to init + run the OFFICIAL Tank/Uhaul Autonomous.");
        telemetry.log().add("after finishing here!");
        telemetry.log().add("");
        telemetry.log().add("When you press a button to answer, it might take a bit,");
        telemetry.log().add("*ALL YOU NEED TO DO IS*");
        telemetry.log().add("Hold the button down until you see \"Release\"");
        telemetry.log().add("Thanks, good luck guys!");


        telemetry.log().add("Waiting for play button pressed...");

        // Wait until we're told to go
        while (!isStarted()) {
            telemetry.update();
            idle();
        }


        telemetry.log().add("...let's begin...");

        telemetry.log().clear();


        // currentDriver = telemetry.addData("Current Driver", alliance.toString());

        currentQuery = telemetry.addData("-- ", message);



        allianceConfig = new AllianceConfig();
        optionsConfig = new OptionsConfig();



        File allianceFile = AppUtil.getInstance().getSettingsFile(allianceFileName);
        File optionsFile = AppUtil.getInstance().getSettingsFile(optionsFileName);

        while(opModeIsActive()){

            if(stepNumber == 0 && buttonPressed1()){

                stepNumber++;

                while(buttonPressed1()){
                    telemetry.update();
                    idle();
                }

            }

            if(stepNumber == 1 && buttonPressed1()){

                registerAllianceButton(gamepad1, allianceConfig.redAlliance);

                currentQuery.setValue("Release");

                while(buttonPressed1()){
                    telemetry.update();
                    idle();
                }

                stepNumber++;
                currentQuery.setValue(message);

            }

            if(stepNumber == 2 && buttonPressed1()){
                registerTasks(gamepad1, optionsConfig.tasks);

                currentQuery.setValue("Release");

                while(buttonPressed1()){
                    telemetry.update();
                    idle();
                }

                stepNumber++;
                currentQuery.setValue(message);
            }
            if(stepNumber == 3 && buttonPressed1()){
                registerMiddleOrWall(gamepad1, optionsConfig.park_middle_or_wall);


                currentQuery.setValue("Release");

                while(buttonPressed1()){
                    telemetry.update();
                    idle();
                }

                stepNumber++;
                currentQuery.setValue(message);

                if(gamepad1.a){

                    ReadWriteFile.writeFile(allianceFile, serializeAllianceConfig());
                    ReadWriteFile.writeFile(optionsFile, serializeOptionsConfig());
                }



                stepNumber++;
            }

            //
            //    If we wanted to add more options, we can use the code below
            //
           /*
            if(stepNumber == 4 && buttonPressed1()){
                registerButton(gamepad1, manipulatorControls.lift_down);


                currentQuery.setValue("Release");

                while(buttonPressed1()){
                    telemetry.update();
                    idle();
                }

                stepNumber++;
                currentQuery.setValue(message);
            }

            if(stepNumber == 5 && buttonPressed1()){
                registerButton(gamepad1, manipulatorControls.claw_open);


                currentQuery.setValue("Release");

                while(buttonPressed1()){
                    telemetry.update();
                    idle();
                }

                stepNumber++;
                currentQuery.setValue(message);
            }
            if(stepNumber == 6 && buttonPressed1()){
                registerButton(gamepad1, manipulatorControls.claw_closed);


                currentQuery.setValue("Release");

                while(buttonPressed1()){
                    telemetry.update();
                    idle();
                }

                stepNumber++;
                currentQuery.setValue(message);
            }
            if(stepNumber == 7 && buttonPressed1()){
                registerButton(gamepad1, manipulatorControls.arm_down);


                currentQuery.setValue("Release");

                while(buttonPressed1()){
                    telemetry.update();
                    idle();
                }

                stepNumber++;
                currentQuery.setValue(message);
            }
            if(stepNumber == 8 && buttonPressed1()){
                registerButton(gamepad1, manipulatorControls.arm_up);


                currentQuery.setValue("Release");

                while(buttonPressed1()){
                    telemetry.update();
                    idle();
                }

                stepNumber++;
                currentQuery.setValue(message);
            }
            if(stepNumber == 9 && (gamepad1.left_stick_y != 0.0 || gamepad1.right_stick_y != 0.0)){
                if(gamepad1.left_stick_y != 0.0) manipulatorControls.foundation_movers = Trigger.LEFT_STICK_Y;
                else if(gamepad1.right_stick_y != 0.0) manipulatorControls.foundation_movers = Trigger.RIGHT_STICK_Y;


                currentQuery.setValue("Release");

                while(gamepad1.left_stick_y != 0.0 || gamepad1.right_stick_y != 0.0){
                    telemetry.update();
                    idle();
                }

                stepNumber++;
                currentQuery.setValue(message);
            }


            if(stepNumber == 4 && buttonPressed1()){
                registerButton(gamepad1, manipulatorControls.arm_up);

                if(gamepad1.a){

                    ReadWriteFile.writeFile(driverFile, serializeDriverControls());
                    ReadWriteFile.writeFile(manipulatorFile, serializeManipulatorControls());
                }



                stepNumber++;
            }
*/


            telemetry.update();
        }






    }


    /**
     * Puts the alliance config into a Json file
     */
    private String serializeAllianceConfig(){

        return SimpleGson.getInstance().toJson(allianceConfig);
    }

    /**
     * Puts the options config into a Json file
     */
    private String serializeOptionsConfig(){

        return SimpleGson.getInstance().toJson(optionsConfig);
    }

    /**
     * After button1 is pressed returns gamepad values
     */
    private boolean buttonPressed1(){
        return (gamepad1.right_bumper || gamepad1.left_bumper || gamepad1.a || gamepad1.x || gamepad1.y ||
                gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.left_stick_button
                || gamepad1.b || gamepad1.right_stick_button);
    }



    /** Maps gamepad inputs to the options */
    private void registerTasks(Gamepad gamepad, Options mapTo){
        if(gamepad.a) mapTo = Options.DO_FOUNDATION;
        else if (gamepad.b) mapTo = Options.DO_STONE;
        else if(gamepad.x) mapTo = Options.DO_BOTH;
        else if(gamepad.y) mapTo = Options.DO_NEITHER;
        //else if(gamepad.left_bumper) mapTo = Button.LEFT_BUMPER;
        //else if(gamepad.right_bumper) mapTo = Button.RIGHT_BUMPER;
        //else if(gamepad.left_stick_button) mapTo = Button.LEFT_STICK_BUTTON;
        //else if(gamepad.right_stick_button) mapTo = Button.RIGHT_STICK_BUTTON;
        //else if(gamepad.dpad_up) mapTo = Button.DPAD_UP;
        //else if(gamepad.dpad_down) mapTo = Button.DPAD_DOWN;
        //else if(gamepad.dpad_left) mapTo = Button.DPAD_LEFT;
        //else if(gamepad.dpad_right) mapTo = Button.DPAD_RIGHT;
    }
    /** Maps gamepad inputs to the options */
    private void registerMiddleOrWall(Gamepad gamepad, Options mapTo){
        if(gamepad.a) mapTo = Options.PARK_MIDDLE;
        else if (gamepad.b) mapTo = Options.PARK_WALL;
        //else if(gamepad.x) mapTo = Button.X;
        //else if(gamepad.y) mapTo = Button.Y;
        //else if(gamepad.left_bumper) mapTo = Button.LEFT_BUMPER;
        //else if(gamepad.right_bumper) mapTo = Button.RIGHT_BUMPER;
        //else if(gamepad.left_stick_button) mapTo = Button.LEFT_STICK_BUTTON;
        //else if(gamepad.right_stick_button) mapTo = Button.RIGHT_STICK_BUTTON;
        //else if(gamepad.dpad_up) mapTo = Button.DPAD_UP;
        //else if(gamepad.dpad_down) mapTo = Button.DPAD_DOWN;
        //else if(gamepad.dpad_left) mapTo = Button.DPAD_LEFT;
        //else if(gamepad.dpad_right) mapTo = Button.DPAD_RIGHT;
    }
    /** Selects either the red or blue alliance */
    private void registerAllianceButton(Gamepad gamepad, Alliance mapTo){
        if(gamepad.b) mapTo = Alliance.RED;
        else if (gamepad.x) mapTo = Alliance.BLUE;
        //else if(gamepad.x) mapTo = Button.X;
        //else if(gamepad.y) mapTo = Button.Y;
        //else if(gamepad.left_bumper) mapTo = Button.LEFT_BUMPER;
        //else if(gamepad.right_bumper) mapTo = Button.RIGHT_BUMPER;
        //else if(gamepad.left_stick_button) mapTo = Button.LEFT_STICK_BUTTON;
        //else if(gamepad.right_stick_button) mapTo = Button.RIGHT_STICK_BUTTON;
        //else if(gamepad.dpad_up) mapTo = Button.DPAD_UP;
        //else if(gamepad.dpad_down) mapTo = Button.DPAD_DOWN;
        //else if(gamepad.dpad_left) mapTo = Button.DPAD_LEFT;
        //else if(gamepad.dpad_right) mapTo = Button.DPAD_RIGHT;
    }


    /**
     * List all the messages that appear.
     */
    private static final String[] queries = new String[]{
            "To ensure the controls are working, press A to continue.",
            "Press \"B\" for RED ALLIANCE, and press \"X\" for BLUE ALLIANCE",
            "What tasks should the robot do? Press \"A\" for FOUNDATION ONLY, press \"B\" for STONE ONLY, press \"X\" for BOTH, and press \"Y\" for NEITHER",
            "Should the robot park closer to the middle Skybridge, or the wall? Press \"A\" for MIDDLE SIDE, and press \"B\" for WALL SIDE",
            "Done! Please press \"A\" to continue (Don't exit yet!).",
            "File Saved! Now press stop and init the OFFICIAL Autonomous!"
    };


    /**
     * Deserialize the json file for alliance
     */
    public static AllianceConfig deserializeAlliance(){

        File allianceFile = AppUtil.getInstance().getSettingsFile(allianceFileName);
        String data = ReadWriteFile.readFile(allianceFile);
        return SimpleGson.getInstance().fromJson(data, AllianceConfig.class);
    }

    /**
     * Deserialize the json file for other options
     */
    public static OptionsConfig deserializeOptions(){

        File optionsFile = AppUtil.getInstance().getSettingsFile(optionsFileName);
        String data = ReadWriteFile.readFile(optionsFile);
        return SimpleGson.getInstance().fromJson(data, OptionsConfig.class);
    }
}
