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

    private final Servo leftLockingServo;
    private final Servo rightLockingServo;

    public SuspensionLiftComponent(HardwareMap hardwareMap) {

        this.suspensionLift = hardwareMap.get(DcMotorEx.class, HardwareMapIdentities.SUSPENSION_LIFT);
        this.suspensionLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.suspensionLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.suspensionLiftGuideServo = hardwareMap.get(Servo.class, HardwareMapIdentities.SUSPENSION_LIFT_GUIDE);
        this.suspensionLiftGuideServo.setDirection(Servo.Direction.REVERSE);

        this.leftLockingServo = hardwareMap.get(Servo.class, HardwareMapIdentities.LEFT_LOCKING_SERVO);
        this.rightLockingServo = hardwareMap.get(Servo.class, HardwareMapIdentities.RIGHT_LOCKING_SERVO);

    }

    public void initializeServos() {

        unlockLiftLocks();

    }

    public void setPower(double power) {

        double scaledRange = (power + 1.0) / 2.0;

        this.suspensionLiftGuideServo.setPosition(scaledRange);
        this.suspensionLift.setVelocity(MOTOR_VELOCITY * power);

    }

    public void unlockLiftLocks() {

        this.leftLockingServo.setPosition(0);
        this.rightLockingServo.setPosition(0);

    }

    public void lockLiftServos() {

        this.leftLockingServo.setPosition(1);
        this.rightLockingServo.setPosition(1);

    }

}
