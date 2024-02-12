package com.bravenatorsrobotics.components;

import com.bravenatorsrobotics.HardwareMapIdentities;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.spherical.twod.SphericalPolygonsSet;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SuspensionLiftComponent {

    public static double SUSPENSION_MOTOR_VELOCITY = 300;
    public static int RISE_ENCODER_POSITION = 700;
    public static int SUSPEND_HEIGHT = 300;

    // TODO: ADD VARIABLES
    public static final double SPOOL_DIAMETER = 5; // inches
    public static final double SPOOL_CIRCUMFERENCE = SPOOL_DIAMETER * Math.PI;
    public static final double INCHES_PER_MOTOR_REV = SPOOL_CIRCUMFERENCE;

    private final DcMotorEx suspensionLift;

    public SuspensionLiftComponent(HardwareMap hardwareMap) {

        this.suspensionLift = hardwareMap.get(DcMotorEx.class, HardwareMapIdentities.SUSPENSION_LIFT);
        this.suspensionLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.suspensionLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.suspensionLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void update() {


    }

    public void setVelocity(double velocity) {

        this.suspensionLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.suspensionLift.setVelocity(velocity);

    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Lift Position", this.suspensionLift.getCurrentPosition());
    }

}
