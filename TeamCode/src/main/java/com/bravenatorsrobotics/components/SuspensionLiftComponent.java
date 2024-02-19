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

    private final DcMotorEx suspensionLift;

    public SuspensionLiftComponent(HardwareMap hardwareMap) {

        this.suspensionLift = hardwareMap.get(DcMotorEx.class, HardwareMapIdentities.SUSPENSION_LIFT);
        this.suspensionLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.suspensionLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.suspensionLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void initializeServos() {

    }

    public void setManualPower(double power) {


        double scaledRange = (power + 1.0) / 2.0;

        this.suspensionLift.setVelocity(MOTOR_VELOCITY * power);

    }

    //    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime servoTimer = new ElapsedTime();

    public void update() {


    }

    public void runLockSequence() {


    }

    public void unlockLiftLocks() {

    }

    public void lockLiftServos() {

    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Lift Pos", this.suspensionLift.getCurrentPosition());
    }
}