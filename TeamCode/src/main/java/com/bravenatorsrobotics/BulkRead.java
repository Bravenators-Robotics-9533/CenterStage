package com.bravenatorsrobotics;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public class BulkRead {

    public enum Mode {
        AUTOMATIC,
        MANUAL
    }

    private static final ArrayList<LynxModule> lynxModules = new ArrayList<>();
    private static Mode activeMode = Mode.AUTOMATIC;

    public static void clearHubCache() {
        if(activeMode != Mode.MANUAL)
            return;

        lynxModules.forEach((LynxModule::clearBulkCache));
    }

    public static void setMode(HardwareMap hardwareMap, Mode mode) {

        lynxModules.clear();
        lynxModules.addAll(hardwareMap.getAll(LynxModule.class));
        lynxModules.forEach((lynxModule -> lynxModule.setBulkCachingMode(mode == Mode.AUTOMATIC ? LynxModule.BulkCachingMode.AUTO : LynxModule.BulkCachingMode.MANUAL)));

        BulkRead.activeMode = mode;

    }

}
