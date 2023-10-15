package com.bravenatorsrobotics.components;

import com.bravenatorsrobotics.HardwareMapIdentities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftComponent {

    public static final int LIFT_POSITION_INTAKE = 0;
    public static final int LIFT_POSITION_ARM_SAFE = 100;

    public static final int LIFT_POSITION_STAGE_LOWER_RELEASE = 200;
    public static final int LIFT_POSITION_STAGE_UPPER_RELEASE = 300;

    private final DcMotorEx liftMotor;

    public LiftComponent(HardwareMap hardwareMap) {

        liftMotor = hardwareMap.get(DcMotorEx.class, HardwareMapIdentities.LIFT);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void moveByPower(double power) {

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setPower(power);

    }

    public void goToEncoderPosition(int encoderPosition, double power) {

        liftMotor.setTargetPosition(encoderPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor.setPower(power);

    }

}
