package com.bravenatorsrobotics.components;

import com.bravenatorsrobotics.HardwareMapIdentities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SwingArmComponent {

    public static final double SWING_ARM_MOTOR_SPEED_OUT = 0.5;
    public static final double SWING_ARM_MOTOR_SPEED_IN = 0.25;

    public static final int SWING_ARM_INTAKE_POSITION = 0;
    public static final int SWING_ARM_SCORING_POSITION = 750;

    private static final int SWING_ARM_TOLERANCE = 10;

    private final DcMotorEx swingArmMotor;

    private enum State {

        INTAKE,
        SCORING

    }

    private State currentState = State.INTAKE;

    public SwingArmComponent(HardwareMap hardwareMap) {

        this.swingArmMotor = hardwareMap.get(DcMotorEx.class, HardwareMapIdentities.SWING_ARM);
        this.swingArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.swingArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.swingArmMotor.setTargetPositionTolerance(SWING_ARM_TOLERANCE);

        this.swingArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.swingArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void update() {

        switch (this.currentState) {

            case INTAKE:
                this.swingArmMotor.setTargetPosition(SWING_ARM_INTAKE_POSITION);
                this.swingArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.swingArmMotor.setPower(SWING_ARM_MOTOR_SPEED_IN);

                break;

            case SCORING:
                this.swingArmMotor.setTargetPosition(SWING_ARM_SCORING_POSITION);
                this.swingArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.swingArmMotor.setPower(SWING_ARM_MOTOR_SPEED_OUT);

                break;

        }

    }

    public void setPower(double power) {
        this.swingArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.swingArmMotor.setPower(power);
    }

    public void goToScoringPosition() {

        this.currentState = State.SCORING;

    }

    public void goToIntakePosition() {

        this.currentState = State.INTAKE;

    }

    public int getTargetPosition() { return this.swingArmMotor.getTargetPosition(); }

    public int getSwingArmMotorPosition() { return this.swingArmMotor.getCurrentPosition(); }

    public boolean isMotorBusy() { return this.swingArmMotor.isBusy(); }

    public void printTelemetry(Telemetry telemetry) {

        telemetry.addData("Swing Arm Motor Position", this.swingArmMotor.getCurrentPosition());

    }

}
