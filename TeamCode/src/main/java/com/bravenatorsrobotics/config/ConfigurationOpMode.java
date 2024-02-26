package com.bravenatorsrobotics.config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Configuration", group="Config")
public class ConfigurationOpMode extends LinearOpMode {

    private static final SimpleMenu MENU = new SimpleMenu("Configuration Menu");

    private static final String SINGLE_CONTROLLER_OVERRIDE = "Single Controller Override";
    private static final String STARTING_POSITION = "Starting Position";
    private static final String AUTONOMOUS_WAIT_TIME = "Autonomous Wait Time";

    @Override
    public void runOpMode() {
        Config config = new Config(hardwareMap.appContext);

        MENU.clearOptions();

        MENU.addOption(STARTING_POSITION, Config.StartingPosition.class, config.GetStartingPosition());
//        MENU.addOption(SINGLE_CONTROLLER_OVERRIDE, config.IsSingleControllerOverride());
        MENU.addOption(AUTONOMOUS_WAIT_TIME, config.GetAutonomousWaitTime(), -1.0, 1.0);

        MENU.setGamepad(gamepad1);
        MENU.setTelemetry(telemetry);

        waitForStart();

        while(!isStopRequested()) {
            MENU.displayMenu();

//            config.SetIsControllerOverride(Boolean.parseBoolean(MENU.getCurrentChoiceOf(SINGLE_CONTROLLER_OVERRIDE)));
            config.SetStartingPosition(Config.StartingPosition.toStartingPosition(MENU.getCurrentChoiceOf(STARTING_POSITION)));
            config.SetAutonomousWaitTime(Float.parseFloat(MENU.getCurrentChoiceOf(AUTONOMOUS_WAIT_TIME)));

            sleep(50); // Keep the processor from dying
        }

        config.Save();
    }
}