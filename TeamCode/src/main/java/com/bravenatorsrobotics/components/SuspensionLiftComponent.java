package com.bravenatorsrobotics.components;

import com.bravenatorsrobotics.HardwareMapIdentities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.net.HttpRetryException;

public class SuspensionLiftComponent {

    private static final double SERVO_VELOCITY = 1.2821; // rev/s.
    private static final double MOTOR_VELOCITY = (1680 * SERVO_VELOCITY) / 2;

    private static final double SUSPENSION_LIFT_POWER = 0.75;

    private final DcMotorEx suspensionLift;
    private final Servo suspensionLiftGuideServo;

    public SuspensionLiftComponent(HardwareMap hardwareMap) {

        this.suspensionLift = hardwareMap.get(DcMotorEx.class, HardwareMapIdentities.SUSPENSION_LIFT);
        this.suspensionLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.suspensionLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.suspensionLiftGuideServo = hardwareMap.get(Servo.class, HardwareMapIdentities.SUSPENSION_LIFT_GUIDE);
        this.suspensionLiftGuideServo.setDirection(Servo.Direction.REVERSE);

    }

    public void setPower(double power) {

//        this.suspensionLift.setVelocity(SERVO_VELOCITY);
        double scaledRange = (power + 1.0) / 2.0;

        this.suspensionLiftGuideServo.setPosition(scaledRange);
        this.suspensionLift.setVelocity(MOTOR_VELOCITY * power);

    }

}
