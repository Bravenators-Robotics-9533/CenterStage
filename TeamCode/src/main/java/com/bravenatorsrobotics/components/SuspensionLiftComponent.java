package com.bravenatorsrobotics.components;

import com.acmerobotics.dashboard.config.Config;
import com.bravenatorsrobotics.HardwareMapIdentities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class SuspensionLiftComponent {

    private static final double MOTOR_VELOCITY = 5200;

    public static double LEFT_ARM_POSITION = 0.75;
    public static double RIGHT_ARM_POSITION = 0.5;

    public static double LEFT_UP_ARM_POSITION = 0.7;
    public static double RIGHT_UP_ARM_POSITION = 0.6;


    private final DcMotorEx suspensionLift;
    private final Servo leftArm;
    private final Servo rightArm;

    public enum State {

        STANDBY,
        RAISING_HOOKS,
        RAISED_STANDBY

    }

    private State state = State.STANDBY;

    public SuspensionLiftComponent(HardwareMap hardwareMap) {

        this.suspensionLift = hardwareMap.get(DcMotorEx.class, HardwareMapIdentities.SUSPENSION_LIFT);
        this.suspensionLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.suspensionLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.suspensionLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.leftArm = hardwareMap.get(Servo.class, HardwareMapIdentities.LEFT_ARM);
        this.rightArm = hardwareMap.get(Servo.class, HardwareMapIdentities.RIGHT_ARM);

        this.leftArm.setDirection(Servo.Direction.REVERSE);
        this.rightArm.setDirection(Servo.Direction.FORWARD);

    }

    public void setManualPower(double power) {

        if(this.state == State.RAISED_STANDBY)
            this.suspensionLift.setVelocity(MOTOR_VELOCITY * power);

    }

    public void init() {
        this.leftArm.setPosition(0);
        this.rightArm.setPosition(0);
    }

    public void update() {

        switch (this.state) {

            case RAISING_HOOKS -> {

                this.leftArm.setPosition(LEFT_ARM_POSITION);
                this.rightArm.setPosition(RIGHT_ARM_POSITION);
                this.state = State.RAISED_STANDBY;

            }

        }

    }

    public void moveHooks() {

        this.leftArm.setPosition(LEFT_UP_ARM_POSITION);
        this.rightArm.setPosition(RIGHT_UP_ARM_POSITION);

    }

    public void raiseHooks() {

        if(this.state == State.STANDBY)
            this.state = State.RAISING_HOOKS;

    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Lift Pos", this.suspensionLift.getCurrentPosition());
    }

    public State getState() { return this.state; }

}
