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

    private static final double SERVO_VELOCITY = 0.5850631; // rev/s. 1.2821 MAX
    private static final double MOTOR_VELOCITY = 5200;

    public static int MOTOR_TARGET_POSITION = 500;

    private final DcMotorEx suspensionLift;
    private final Servo scissorLift;

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

        this.scissorLift = hardwareMap.get(Servo.class, HardwareMapIdentities.SCISSOR_LIFT);

    }

    public void setManualPower(double power) {

        double scissorServoPosition = ((SERVO_VELOCITY * power) + 1.0) / 2.0;

        this.suspensionLift.setVelocity(MOTOR_VELOCITY * power);
        this.scissorLift.setPosition(scissorServoPosition);

    }

    public double getInstantaneousScissorServoPosition() {

        return (((this.suspensionLift.getVelocity() * SERVO_VELOCITY) / MOTOR_VELOCITY) + 1.0) / 2.0;

    }

    public void update() {

        switch (this.state) {

            case RAISING_HOOKS -> {

                if(this.suspensionLift.getTargetPosition() != MOTOR_TARGET_POSITION) {
                   this.suspensionLift.setTargetPosition(MOTOR_TARGET_POSITION);
                   this.suspensionLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                   this.suspensionLift.setVelocity(MOTOR_VELOCITY);
                } else if(this.suspensionLift.isBusy()) {
                    this.scissorLift.setPosition(getInstantaneousScissorServoPosition());
                } else {
                    this.scissorLift.setPosition(0.5); // STOP
                    this.state = State.RAISED_STANDBY;
                }

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
