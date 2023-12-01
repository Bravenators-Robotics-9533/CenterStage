package com.bravenatorsrobotics.components;

import com.bravenatorsrobotics.HardwareMapIdentities;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AirplaneLauncher {

    private final Servo launcherServo;

    public AirplaneLauncher(HardwareMap hardwareMap) {

        this.launcherServo = hardwareMap.get(Servo.class, HardwareMapIdentities.AIRPLANE_LAUNCHER_SERVO);

    }

    public void initializeServo() {

        this.launcherServo.setPosition(0);

    }

    public void launch() {

        this.launcherServo.setPosition(1);

    }

}
