package com.bravenatorsrobotics.config;

import android.content.Context;
import android.content.SharedPreferences;

public class Config {

    private static final String PREFERENCES_ID = "RobotPrefCenterStage9533";
    private SharedPreferences sp;

    private static final String SINGLE_CONTROLLER_OVERRIDE = "SingleControllerOverride";
    private static boolean singleControllerOverride;

    private static final String AUTONOMOUS_WAIT_TIME = "AutonomousWaitTime";
    private static float autonomousWaitTimeSeconds;

    private static final String STARTING_POSITION = "STARTING_POSITION";
    private static StartingPosition startingPosition;
    public enum StartingPosition {
        RED, BLUE;

        public static StartingPosition toStartingPosition(String textPosition) {
            try {
                return valueOf(textPosition);
            } catch(Exception e) {
                return RED; // TODO: Throw Warning
            }
        }
    }

    public Config(Context context) {
        sp = context.getSharedPreferences(PREFERENCES_ID, Context.MODE_PRIVATE);
        Load();
    }

    public void Load() {
        singleControllerOverride = sp.getBoolean(SINGLE_CONTROLLER_OVERRIDE, false);
        startingPosition = StartingPosition.toStartingPosition(sp.getString(STARTING_POSITION, StartingPosition.RED.name()));
        autonomousWaitTimeSeconds = sp.getFloat(AUTONOMOUS_WAIT_TIME, 0.0f);
    }

    public void Save() {
        SharedPreferences.Editor editor = sp.edit();

        editor.putBoolean(SINGLE_CONTROLLER_OVERRIDE, singleControllerOverride);
        editor.putString(STARTING_POSITION, startingPosition.name());
        editor.putFloat(AUTONOMOUS_WAIT_TIME, autonomousWaitTimeSeconds);

        editor.apply();
    }

    public void SetIsControllerOverride(boolean value) { Config.singleControllerOverride = value; }
    public boolean IsSingleControllerOverride() { return singleControllerOverride; }

    public void SetStartingPosition(StartingPosition startingPosition) { Config.startingPosition = startingPosition; }
    public StartingPosition GetStartingPosition() { return startingPosition; }

    public float GetAutonomousWaitTime() { return autonomousWaitTimeSeconds; }
    public void SetAutonomousWaitTime(float waitTime) { autonomousWaitTimeSeconds = waitTime; }

}