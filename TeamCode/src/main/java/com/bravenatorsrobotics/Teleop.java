package com.bravenatorsrobotics;

import com.bravenatorsrobotics.components.IntakeComponent;
import com.bravenatorsrobotics.components.LiftComponent;
import com.bravenatorsrobotics.components.PixelPouchComponent;
import com.bravenatorsrobotics.gamepad.FtcGamePad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import roadrunner.drive.MecanumDrive;

@TeleOp(name="Teleop", group="Competition")
public class Teleop extends LinearOpMode {

    private static final double MAX_ROBOT_SPEED = 1.0;
    private static final double SLOW_MODE_SPEED = 0.2;

    private FtcGamePad driverGamePad;
    private FtcGamePad operatorGamePad;

    private MecanumDrive drive;

    private IntakeComponent intakeComponent;
    private PixelPouchComponent pixelPouchComponent;
    private LiftComponent liftComponent;

    private boolean isSlowModeActive = false;
    private boolean shouldUseMasterController = false;

    private double offsetHeading = 0;

    private void Initialize() {

        this.driverGamePad   = new FtcGamePad("Driver", gamepad1, this::OnDriverGamePadChange);
        this.operatorGamePad = new FtcGamePad("Operator", gamepad2, this::OnOperatorGamePadChange);

        this.drive = new MecanumDrive(super.hardwareMap);
        this.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.intakeComponent = new IntakeComponent(super.hardwareMap);

        this.pixelPouchComponent = new PixelPouchComponent(super.hardwareMap);
        this.pixelPouchComponent.addOnClampCallback(this::triggerPixelClampedRumble);
        this.pixelPouchComponent.initializeServo();

        this.liftComponent = new LiftComponent(super.hardwareMap);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        Initialize();

        telemetry.addData("Status", "Initialized!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Pixel Pouch Status", pixelPouchComponent.getPixelPouchStatus());
            telemetry.update();

            this.handleDrive();
            this.handleLift();

            this.pixelPouchComponent.update();

            driverGamePad.update();
            operatorGamePad.update();

        }
    }

    private void OnDriverGamePadChange(FtcGamePad gamepad, int button, boolean isPressed) {

        switch (button) {

            case FtcGamePad.GAMEPAD_BACK:
                if(isPressed)
                    offsetHeading = drive.getRawExternalHeading();

                break;

            case FtcGamePad.GAMEPAD_RBUMPER:
                if(isPressed)
                    isSlowModeActive = !isSlowModeActive;

                break;

        }

    }

    private void OnOperatorGamePadChange(FtcGamePad gamepad, int button, boolean isPressed) {

        if(shouldUseMasterController) // Forward Controls to Driver
            this.OnDriverGamePadChange(gamepad, button, isPressed);

        boolean shouldStopIntake = true;

        switch(button) {

            case FtcGamePad.GAMEPAD_A:
                if(isPressed) {
                    intakeComponent.runIntakeForward();
                    shouldStopIntake = false;
                }

                break;

            case FtcGamePad.GAMEPAD_Y:
                if(isPressed) {
                    intakeComponent.runIntakeBackwards();
                    shouldStopIntake = false;
                }

                break;

            case FtcGamePad.GAMEPAD_B:
                if(isPressed) {
                    pixelPouchComponent.togglePouchPosition();
                }

                break;

            case FtcGamePad.GAMEPAD_X:
                if(isPressed)
                    pixelPouchComponent.requestRelease();

                break;

        }

        if(shouldStopIntake && intakeComponent.isRunning()) {
            this.intakeComponent.stopIntakeMotor();
        }

    }

    private void handleDrive() {

        double y = Range.clip(Math.pow(-gamepad1.left_stick_y, 3), -1.0, 1.0);
        double xt = (Math.pow(gamepad1.right_trigger, 3) - Math.pow(gamepad1.left_trigger, 3)) * (shouldUseMasterController ? 0 : 1);
        double x = Range.clip(Math.pow(gamepad1.left_stick_x, 3) + xt, -1.0, 1.0) * 1.1;
        double rx = Range.clip(Math.pow(gamepad1.right_stick_x, 3), -1.0, 1.0);

        // Read inverse IMU heading, as the UMG heading is CW positive
        double botHeading = -drive.getRawExternalHeading() + offsetHeading;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        if(isSlowModeActive) {

            frontLeftPower  = Range.clip(frontLeftPower, -SLOW_MODE_SPEED, SLOW_MODE_SPEED);
            frontRightPower = Range.clip(frontRightPower, -SLOW_MODE_SPEED, SLOW_MODE_SPEED);
            backLeftPower   = Range.clip(backLeftPower, -SLOW_MODE_SPEED, SLOW_MODE_SPEED);
            backRightPower  = Range.clip(backRightPower, -SLOW_MODE_SPEED, SLOW_MODE_SPEED);

        } else {

            frontLeftPower  = Range.clip(frontLeftPower, -MAX_ROBOT_SPEED, MAX_ROBOT_SPEED);
            frontRightPower = Range.clip(frontRightPower, -MAX_ROBOT_SPEED, MAX_ROBOT_SPEED);
            backLeftPower   = Range.clip(backLeftPower, -MAX_ROBOT_SPEED, MAX_ROBOT_SPEED);
            backRightPower  = Range.clip(backRightPower, -MAX_ROBOT_SPEED, MAX_ROBOT_SPEED);

        }

        drive.setMotorPowers(frontLeftPower, backLeftPower, backRightPower, frontRightPower);

    }

    private void handleLift() {

        Gamepad gamepad = shouldUseMasterController ? gamepad1 : gamepad2;

        double manualLiftPower = Range.clip(Math.pow(gamepad.right_trigger - gamepad.left_trigger, 3), -1.0, 1.0);
        liftComponent.moveByPower(manualLiftPower);

    }

    private void triggerPixelClampedRumble() {

        this.gamepad1.rumbleBlips(2);
        this.gamepad2.rumbleBlips(2);

    }

}
