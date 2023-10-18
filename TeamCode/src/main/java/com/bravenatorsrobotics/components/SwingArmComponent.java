package com.bravenatorsrobotics.components;

import com.bravenatorsrobotics.HardwareMapIdentities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SwingArmComponent {

    public static final double SWING_ARM_MAX_VELOCITY = 125; // RPM

    public static final double SWING_ARM_MOTOR_SPEED = 1.0;

    public static final int SWING_ARM_ZERO_POSITION = 0;
    public static final int SWING_ARM_OUT_POSITION = 500;

    private final DcMotorEx swingArmMotor;

    public SwingArmComponent(HardwareMap hardwareMap) {

        this.swingArmMotor = hardwareMap.get(DcMotorEx.class, HardwareMapIdentities.SWING_ARM);

        this.swingArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.swingArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void setPower(double power) {
        this.swingArmMotor.setVelocity(power * SWING_ARM_MAX_VELOCITY);
    }

    public void swingOut() {

        this.swingArmMotor.setTargetPosition(SWING_ARM_OUT_POSITION);
        this.swingArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.setPower(SWING_ARM_MOTOR_SPEED);

    }

    public void goToIntakePosition() {

        this.swingArmMotor.setTargetPosition(SWING_ARM_ZERO_POSITION);
        this.swingArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.setPower(SWING_ARM_MOTOR_SPEED);

    }

    public boolean isBusy() { return this.swingArmMotor.isBusy(); }

}
