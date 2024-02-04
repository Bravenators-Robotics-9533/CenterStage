package com.bravenatorsrobotics.components;

import com.acmerobotics.dashboard.config.Config;
import com.bravenatorsrobotics.HardwareMapIdentities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class SwingArmComponent {

    public static final int SWING_ARM_SAFE_INTAKE_POSITION = 600;
    public static final int SWING_ARM_SAFE_SCORING_POSITION = 450;

    public static final double SWING_ARM_MOTOR_SPEED_FLIP = 0.5;
    public static final double SWING_ARM_MOTOR_SPEED_OUT = 0.85;
    public static final double SWING_ARM_MOTOR_SPEED_IN = 0.2;

    public static final int SWING_ARM_INTAKE_POSITION = 0;
    public static final int SWING_ARM_SCORING_POSITION = 750;

    private static final int SWING_ARM_TOLERANCE = 10;

    private final DcMotorEx swingArmMotor;
    private int targetPosition = SWING_ARM_INTAKE_POSITION;

    private ElapsedTime timer = new ElapsedTime();

    public static double k_P_OUT = 0.002;
    public static double k_P_IN = 0.0004;

    public static double k_I_OUT = 0.0;
    public static double k_I_IN = 0.0;

    public static double k_D_OUT = 0.0;
    public static double k_D_IN = 0.0;

    private double p, i, d = 0;

    private enum State {

        INTAKE,
        FLIP_POSITION,
        SCORING

    }

    private State currentState = State.INTAKE;

    public SwingArmComponent(HardwareMap hardwareMap) {

        this.swingArmMotor = hardwareMap.get(DcMotorEx.class, HardwareMapIdentities.SWING_ARM);
        this.swingArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.swingArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.swingArmMotor.setTargetPositionTolerance(SWING_ARM_TOLERANCE);

        this.swingArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.swingArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    private static final double MAX_I = 20;
    private double currentTime, currentError, previousError, previousTime, output = 0;

    private int previousTargetPosition = 0;

    private double handlePIDController() {

        this.currentTime = timer.seconds();
        this.currentError = targetPosition - this.swingArmMotor.getCurrentPosition();

        if(this.targetPosition != this.previousTargetPosition)
            this.i = 0;

        if(this.targetPosition == SWING_ARM_SCORING_POSITION) {

            p = k_P_OUT * currentError;

            i += k_I_OUT * (currentError * (currentTime - this.previousTime));
            i = Range.clip(i, -MAX_I, MAX_I);

            d = k_D_OUT * (currentError - previousError) / (currentTime - previousTime);

        } else {

            p = k_P_IN * currentError;

            i += k_I_IN * (currentError * (currentTime - this.previousTime));
            i = Range.clip(i, -MAX_I, MAX_I);

            d = k_D_IN * (currentError - previousError) / (currentTime - previousTime);

        }

        output = p + i + d;
        previousError = currentError;
        previousTime = currentTime;
        previousTargetPosition = targetPosition;

        return output;

    }

    public void update() {

        double input = this.handlePIDController();
        this.swingArmMotor.setPower(input);

    }

    public void setPower(double power) {
        this.swingArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.swingArmMotor.setPower(power);
    }

    public void goToScoringPosition() {

        this.currentState = State.SCORING;
        this.targetPosition = SWING_ARM_SCORING_POSITION;

    }

    public void goToIntakePosition() {

        this.currentState = State.INTAKE;
        this.targetPosition = SWING_ARM_INTAKE_POSITION;

    }

    public int getTargetPosition() { return this.targetPosition; }

    public int getSwingArmMotorPosition() { return this.swingArmMotor.getCurrentPosition(); }

    public boolean isMotorBusy() {
        return Math.abs(this.targetPosition - this.swingArmMotor.getCurrentPosition()) > SWING_ARM_TOLERANCE;
    }

    public void printTelemetry(Telemetry telemetry) {

        telemetry.addData("Swing Arm Motor Position", this.swingArmMotor.getCurrentPosition());

    }

}
