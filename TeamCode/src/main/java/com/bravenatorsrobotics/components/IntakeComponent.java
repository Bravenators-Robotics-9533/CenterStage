package com.bravenatorsrobotics.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeComponent {

    private static final double INTAKE_VELOCITY_RPM = 100;

    private final DcMotorEx intakeMotor;

    public IntakeComponent(HardwareMap hardwareMap) {

        this.intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        this.intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void runIntakeForward() {

        this.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.intakeMotor.setVelocity(INTAKE_VELOCITY_RPM);

    }

    public void runIntakeBackwards() {

        this.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.intakeMotor.setVelocity(-INTAKE_VELOCITY_RPM);

    }

    public boolean isRunning() { return this.intakeMotor.getPower() != 0; }

    public void stopIntakeMotor() { this.intakeMotor.setPower(0); }


}
