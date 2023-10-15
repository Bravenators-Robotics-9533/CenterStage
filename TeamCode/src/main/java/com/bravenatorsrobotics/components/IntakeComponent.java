package com.bravenatorsrobotics.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeComponent {

    private static final double INTAKE_POWER = 0.5;

    private final DcMotorEx intakeMotor;

    public IntakeComponent(HardwareMap hardwareMap) {

        this.intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        this.intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void runIntakeForward() {

        this.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.intakeMotor.setPower(INTAKE_POWER);

    }

    public void runIntakeBackwards() {

        this.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.intakeMotor.setPower(-1.0);

    }

    public boolean isRunning() { return this.intakeMotor.getPower() != 0; }

    public boolean isRunningForward() { return this.intakeMotor.getPower() > 0; }
    public boolean isRunningBackwards() { return this.intakeMotor.getPower() < 0; }

    public void stopIntakeMotor() { this.intakeMotor.setPower(0); }


}
