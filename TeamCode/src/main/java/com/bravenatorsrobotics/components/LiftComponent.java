package com.bravenatorsrobotics.components;

import com.bravenatorsrobotics.HardwareMapIdentities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftComponent {

    private static final double MAX_MOTOR_VELOCITY = 2800;

    public static final double LIFT_SPEED = 1.0;

    public static final int LIFT_POSITION_INTAKE = 0;
    public static final int LIFT_POSITION_ARM_SAFE = 1100;

    public static final int LIFT_POSITION_STAGE_LOWER_RELEASE = 700;
    public static final int LIFT_POSITION_STAGE_MIDDLE_RELEASE = 800;
    public static final int LIFT_POSITION_STAGE_UPPER_RELEASE = 1000;

    private final DcMotorEx liftMotor;

    public LiftComponent(HardwareMap hardwareMap) {

        liftMotor = hardwareMap.get(DcMotorEx.class, HardwareMapIdentities.LIFT);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void moveByPower(double power) {

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setPower(power);

    }

    public void goToEncoderPositionAsync(int encoderPosition, double power) {

        liftMotor.setTargetPosition(encoderPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor.setVelocity(MAX_MOTOR_VELOCITY * power);

    }

    public int getCurrentPosition() { return this.liftMotor.getCurrentPosition(); }

    public boolean isBusy() { return this.liftMotor.isBusy(); }

}
