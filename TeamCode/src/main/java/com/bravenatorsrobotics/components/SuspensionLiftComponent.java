package com.bravenatorsrobotics.components;

import com.bravenatorsrobotics.HardwareMapIdentities;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SuspensionLiftComponent {

    private static final double SERVO_VELOCITY = 1.2821; // rev/s.
    private static final double MOTOR_VELOCITY = (1680 * SERVO_VELOCITY) / 2;

    private static final double SUSPENSION_LIFT_DOWN_POWER = 1.0;

    private static final double SERVO_MOVE_TIME_SECONDS = 2.0;

    private final DcMotorEx suspensionLift;
    private final Servo suspensionLiftGuideServo;

    private final Servo leftLockingServo;
    private final Servo rightLockingServo;

    private final RevTouchSensor magneticLimitSensor;

    private boolean isLockingSequence = false;

    private boolean areServosLocking = false;

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

        this.magneticLimitSensor = hardwareMap.get(RevTouchSensor.class, HardwareMapIdentities.SUSPENSION_LIFT_MAGNETIC_LIMIT_SENSOR);

    }

    public void initializeServos() {

    }

    public void setManualPower(double power) {

        if(isLockingSequence)
            return;

        double scaledRange = (power + 1.0) / 2.0;

        this.suspensionLiftGuideServo.setPosition(scaledRange);
        this.suspensionLift.setVelocity(MOTOR_VELOCITY * power);

    }

//    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime servoTimer = new ElapsedTime();

    public void update() {

        if(isLockingSequence) {

            if(this.magneticLimitSensor.isPressed()) {

                this.suspensionLift.setPower(0);
                this.suspensionLiftGuideServo.setPosition(0);

                this.lockLiftServos();

                this.isLockingSequence = false;
            }

        }

        if(areServosLocking) {
            if(servoTimer.seconds() >= SERVO_MOVE_TIME_SECONDS) {
                this.leftLockingServo.setPosition(0.5);
                this.rightLockingServo.setPosition(0.5);
                this.suspensionLiftGuideServo.setPosition(0.5);
                areServosLocking = false;
            }
        }

    }

    public void runLockSequence() {

        isLockingSequence = true;

        this.suspensionLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.suspensionLift.setPower(-SUSPENSION_LIFT_DOWN_POWER);
        this.suspensionLiftGuideServo.setPosition(-1);

    }

    public void unlockLiftLocks() {

        this.leftLockingServo.setPosition(0);
        this.rightLockingServo.setPosition(0);

        areServosLocking = true;
        servoTimer.reset();

    }

    public void lockLiftServos() {

        this.leftLockingServo.setPosition(1);
        this.rightLockingServo.setPosition(1);

        areServosLocking = true;
        servoTimer.reset();

    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Lift Pos", this.suspensionLift.getCurrentPosition());
    }

    public boolean isLimitSensorTriggered() {
        return this.magneticLimitSensor.isPressed();
    }
}
