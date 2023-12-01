package com.bravenatorsrobotics.components;

import com.bravenatorsrobotics.HardwareMapIdentities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SuspensionLiftComponent {

    private static final double SERVO_VELOCITY = 1.2821; // rev/s.
    private static final double MOTOR_VELOCITY = (1680 * SERVO_VELOCITY) / 2;

    private static final double SUSPENSION_LIFT_POWER = 0.75;

    private final DcMotorEx suspensionLift;
    private final Servo suspensionLiftGuideServo;

    private final Servo leftLockingServo;
    private final Servo rightLockingServo;

    private boolean isLockingSequence = false;
    private boolean resetTimeout = false;

    public SuspensionLiftComponent(HardwareMap hardwareMap) {

        this.suspensionLift = hardwareMap.get(DcMotorEx.class, HardwareMapIdentities.SUSPENSION_LIFT);
        this.suspensionLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.suspensionLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.suspensionLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.suspensionLiftGuideServo = hardwareMap.get(Servo.class, HardwareMapIdentities.SUSPENSION_LIFT_GUIDE);
        this.suspensionLiftGuideServo.setDirection(Servo.Direction.REVERSE);

        this.leftLockingServo = hardwareMap.get(Servo.class, HardwareMapIdentities.LEFT_LOCKING_SERVO);
        this.rightLockingServo = hardwareMap.get(Servo.class, HardwareMapIdentities.RIGHT_LOCKING_SERVO);

        this.leftLockingServo.setDirection(Servo.Direction.FORWARD);
        this.rightLockingServo.setDirection(Servo.Direction.REVERSE);

    }

    public void initializeServos() {

        unlockLiftLocks();

    }

    public void setManualPower(double power) {

        if(isLockingSequence || resetTimeout)
            return;

        double scaledRange = (power + 1.0) / 2.0;

        this.suspensionLiftGuideServo.setPosition(scaledRange);
        this.suspensionLift.setVelocity(MOTOR_VELOCITY * power);

    }

    private ElapsedTime timer = new ElapsedTime();

    public void update() {

        if(isLockingSequence) {

            if(!this.suspensionLift.isBusy()) {

                this.lockLiftServos();
                this.suspensionLiftGuideServo.setPosition(0);

                this.resetTimeout = true;
                timer.reset();

                this.isLockingSequence = false;

            }

        }

        if(resetTimeout) {
            if(timer.seconds() > 2) {
                this.suspensionLift.setPower(0);
                this.suspensionLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                this.resetTimeout = false;
            }
        }

    }

    public void runLockSequence() {

        isLockingSequence = true;

        this.suspensionLift.setTargetPosition(0);
        this.suspensionLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.suspensionLift.setVelocity(MOTOR_VELOCITY);
        this.suspensionLiftGuideServo.setPosition(-1);

    }

    public void unlockLiftLocks() {

        this.leftLockingServo.setPosition(0);
        this.rightLockingServo.setPosition(0);

    }

    public void lockLiftServos() {

        this.leftLockingServo.setPosition(1);
        this.rightLockingServo.setPosition(1);

    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Lift Pos", this.suspensionLift.getCurrentPosition());
    }

}
