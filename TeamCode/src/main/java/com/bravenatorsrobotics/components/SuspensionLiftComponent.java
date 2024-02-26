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

    public static double LEFT_ARM_POSITION = 0.65;
    public static double RIGHT_ARM_POSITION = 0.35;

    public static double LEFT_ARM_UP_POSITION = 1;
    public static double RIGHT_ARM_UP_POSITION = 0.55;

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

        this.suspensionLift.setPower(power);

    }

    public void init() {
        this.leftArm.setPosition(0);
        this.rightArm.setPosition(0);
    }

    public void update() {



    }


    public void raiseHooks() {

        if(this.state == State.STANDBY) {
            this.leftArm.setPosition(LEFT_ARM_POSITION);
            this.rightArm.setPosition(RIGHT_ARM_POSITION);

            this.state = State.RAISED_STANDBY;
        }


    }

    public void clearPole() {

        if(this.state != State.STANDBY) {
            this.leftArm.setPosition(LEFT_ARM_UP_POSITION);
            this.rightArm.setPosition(RIGHT_ARM_UP_POSITION);
        }

    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Lift Pos", this.suspensionLift.getCurrentPosition());
    }

    public State getState() { return this.state; }

}
