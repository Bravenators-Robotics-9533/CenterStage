package com.bravenatorsrobotics.components;

import com.bravenatorsrobotics.HardwareMapIdentities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SwingArmComponent {

    public static final double SWING_ARM_MOTOR_SPEED = 1.0;

    public static final int SWING_ARM_ZERO_POSITION = 0;
    public static final int SWING_ARM_OUT_POSITION = 750;

    private static final int SWING_ARM_TOLERANCE = 10;

    private final DcMotorEx swingArmMotor;

    private enum State {

        INTAKE,
        RELEASE

    }

    private State currentState = State.INTAKE;

    public SwingArmComponent(HardwareMap hardwareMap) {

        this.swingArmMotor = hardwareMap.get(DcMotorEx.class, HardwareMapIdentities.SWING_ARM);
        this.swingArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.swingArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.swingArmMotor.setTargetPositionTolerance(SWING_ARM_TOLERANCE);

        this.swingArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.swingArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void update() {

        switch (this.currentState) {

            case INTAKE:
                this.swingArmMotor.setTargetPosition(SWING_ARM_ZERO_POSITION);
                this.swingArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.swingArmMotor.setPower(SWING_ARM_MOTOR_SPEED);

                break;

            case RELEASE:
                this.swingArmMotor.setTargetPosition(SWING_ARM_OUT_POSITION);
                this.swingArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.swingArmMotor.setPower(SWING_ARM_MOTOR_SPEED);

                break;

        }

    }

    public void setPower(double power) {
        this.swingArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.swingArmMotor.setPower(power);
    }

    public void swingOut() {

        this.currentState = State.RELEASE;

    }

    public void goToIntakePosition() {

        this.currentState = State.INTAKE;

    }

    public boolean isBusy() { return this.swingArmMotor.isBusy(); }

}
