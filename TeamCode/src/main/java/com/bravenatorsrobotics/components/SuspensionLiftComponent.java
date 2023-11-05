package com.bravenatorsrobotics.components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.net.HttpRetryException;

public class SuspensionLiftComponent {

    private static final double SUSPENSION_LIFT_POWER = 0.75;

    private final DcMotorEx suspensionLift;

    public SuspensionLiftComponent(HardwareMap hardwareMap) {
        this.suspensionLift = hardwareMap.get(DcMotorEx.class, "suspensionLift");
    }

    public void setPower(double power) {

        this.suspensionLift.setPower(power);

    }

}
