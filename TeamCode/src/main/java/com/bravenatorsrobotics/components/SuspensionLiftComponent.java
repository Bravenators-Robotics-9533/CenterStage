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

    private static final double SERVO_VELOCITY = 0.5850631; // rev/s. MAX 1.2821
    private static final double MAX_MOTOR_VELOCITY = 2500; // MAX 2580

    private static final double SUSPENSION_LIFT_DOWN_POWER = 1.0;

    private static final double SERVO_MOVE_TIME_SECONDS = 2.0;
    private static final double FINISH_SUSPEND_TIME_SECONDS = 0.75;

    private final DcMotorEx suspensionLift;
    private final Servo suspensionLiftGuideServo;

    private final Servo leftLockingServo;
    private final Servo rightLockingServo;

    private final RevTouchSensor magneticLimitSensor;

    private enum State {

        STANDBY,
        SUSPENDING,
        LOCKING,
        FINISHING_SUSPEND

    }

    private State state = State.STANDBY;

    private boolean isUnlocking = false;

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

    private double calculateServoSpeed() {

        double instantaneousServoVelocity = (SERVO_VELOCITY * this.suspensionLift.getVelocity()) / MAX_MOTOR_VELOCITY;

        return (instantaneousServoVelocity + 1.0) / 2.0;

    }

    public void setManualPower(double power) {

        this.suspensionLift.setVelocity(MAX_MOTOR_VELOCITY * power);
        this.suspensionLiftGuideServo.setPosition(calculateServoSpeed());

    }

    private final ElapsedTime timer = new ElapsedTime();

    public void update() {

        switch (this.state) {

            case SUSPENDING -> {

                if(!this.magneticLimitSensor.isPressed()) {

                    this.suspensionLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    this.suspensionLift.setPower(-SUSPENSION_LIFT_DOWN_POWER);
                    this.suspensionLiftGuideServo.setPosition(-1);

                } else {

                    this.suspensionLift.setPower(0);
                    this.suspensionLiftGuideServo.setPosition(0.5);

                    this.state = State.LOCKING;
                    this.timer.reset();

                }

            }

            case LOCKING -> {

                if(this.timer.seconds() < SERVO_MOVE_TIME_SECONDS) {

                    this.leftLockingServo.setPosition(1);
                    this.rightLockingServo.setPosition(1);

                } else {

                    this.leftLockingServo.setPosition(0.5);
                    this.rightLockingServo.setPosition(0.5);

                    this.state = State.FINISHING_SUSPEND;

                }

            }

            case FINISHING_SUSPEND -> {

                if(this.suspensionLift.getPower() == 0) {
                    this.timer.reset();

                    this.suspensionLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    this.suspensionLift.setVelocity(MAX_MOTOR_VELOCITY);
                    this.suspensionLiftGuideServo.setPosition(1);

                } else if(timer.seconds() > FINISH_SUSPEND_TIME_SECONDS) {

                    this.suspensionLiftGuideServo.setPosition(0.5);
                    this.suspensionLift.setPower(0);
                    this.state = State.STANDBY;

                }

            }

        }

        if(this.isUnlocking) {

            if(this.timer.seconds() >= SERVO_MOVE_TIME_SECONDS) {

                this.leftLockingServo.setPosition(0.5);
                this.rightLockingServo.setPosition(0.5);
                this.isUnlocking = false;

            }

        }

    }

    public void runLockSequence() {

        this.state = State.SUSPENDING;

    }

    public void unlockLiftLocks() {

        this.leftLockingServo.setPosition(0);
        this.rightLockingServo.setPosition(0);

        this.isUnlocking = true;
        timer.reset();

    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Lift Pos", this.suspensionLift.getCurrentPosition());
    }

    public boolean isLimitSensorTriggered() {
        return this.magneticLimitSensor.isPressed();
    }
}
