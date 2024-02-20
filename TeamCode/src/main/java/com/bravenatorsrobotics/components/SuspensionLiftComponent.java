package com.bravenatorsrobotics.components;

import com.bravenatorsrobotics.HardwareMapIdentities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SuspensionLiftComponent {

    private static final double SERVO_VELOCITY = 0.5850631; // rev/s. 1.2821 MAX
    private static final double MOTOR_VELOCITY = 5200;

    private static final double SUSPENSION_LIFT_DOWN_POWER = 1.0;

    private static final double SERVO_MOVE_TIME_SECONDS = 2.0;
    private static final double FINISH_SUSPEND_TIME_SECONDS = 0.75;

    private final DcMotorEx suspensionLift;

    private final Servo leftHookServo;
    private final Servo rightHookServo;

    private enum State {

        STANDBY,
        RAISING_HOOKS,
        RAISED_STANDBY,
        SUSPENDING

    }

    private State state = State.STANDBY;

    public SuspensionLiftComponent(HardwareMap hardwareMap) {

        this.suspensionLift = hardwareMap.get(DcMotorEx.class, HardwareMapIdentities.SUSPENSION_LIFT);
        this.suspensionLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.suspensionLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.suspensionLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        this.leftHookServo = hardwareMap.get(Servo.class, HardwareMapIdentities.LEFT_LOCKING_SERVO);
        this.rightHookServo = hardwareMap.get(Servo.class, HardwareMapIdentities.RIGHT_LOCKING_SERVO);

        this.leftHookServo.setDirection(Servo.Direction.FORWARD);
        this.rightHookServo.setDirection(Servo.Direction.REVERSE);

    }

    public void setManualPower(double power) {

        this.suspensionLift.setVelocity(MOTOR_VELOCITY * power);

    }

    public void update() {

        switch (this.state) {

            case RAISING_HOOKS -> {

                

            }

        }

    }

    public void raiseHooks() {

        if(this.state == State.STANDBY)
            this.state = State.RAISING_HOOKS;

    }

    public void suspend() {

        if(this.state == State.RAISED_STANDBY)
            this.state = State.SUSPENDING;

    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Lift Pos", this.suspensionLift.getCurrentPosition());
    }

}
