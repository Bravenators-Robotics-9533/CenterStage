package com.bravenatorsrobotics;

import android.graphics.Color;

import com.bravenatorsrobotics.components.AirplaneLauncher;
import com.bravenatorsrobotics.components.IntakeComponent;
import com.bravenatorsrobotics.components.LiftComponent;
import com.bravenatorsrobotics.components.PixelPouchComponent;
import com.bravenatorsrobotics.components.SuspensionLiftComponent;
import com.bravenatorsrobotics.components.SwingArmComponent;
import com.bravenatorsrobotics.config.Config;
import com.bravenatorsrobotics.gamepad.FtcGamePad;
import com.bravenatorsrobotics.multiComponentSystem.LiftMultiComponentSystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import roadrunner.drive.MecanumDrive;

@TeleOp(name="Teleop", group="Competition")
@com.acmerobotics.dashboard.config.Config
public class Teleop extends LinearOpMode {

    private static final double MAX_ROBOT_SPEED = 1.0;
    private static final double SLOW_MODE_SPEED = 0.2;

    private FtcGamePad driverGamePad;
    private FtcGamePad operatorGamePad;

    private MecanumDrive drive;

    private IntakeComponent intakeComponent;
    private PixelPouchComponent pixelPouchComponent;
    private LiftComponent liftComponent;
    private SwingArmComponent swingArmComponent;
    private SuspensionLiftComponent suspensionLiftComponent;
    private AirplaneLauncher airplaneLauncher;

    private LiftMultiComponentSystem liftMultiComponentSystem;

    private boolean isSlowModeActive = false;
    private boolean shouldUseMasterController = false;
    private boolean didAutoChangeSlowMode = false;

    private float deltaTime = 0;

    private static double changeHeading = 0;

    private double offsetHeading = 0;

    private void Initialize() {

        Config config = new Config(this.hardwareMap.appContext);
        shouldUseMasterController = config.IsSingleControllerOverride();

//        this.offsetHeading


        this.driverGamePad   = new FtcGamePad("Driver", gamepad1, this::OnDriverGamePadChange);
        this.operatorGamePad = new FtcGamePad("Operator", gamepad2, this::OnOperatorGamePadChange);

        this.drive = new MecanumDrive(super.hardwareMap);
        this.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.intakeComponent = new IntakeComponent(super.hardwareMap);

        this.pixelPouchComponent = new PixelPouchComponent(super.hardwareMap, true);
        this.pixelPouchComponent.addOnClampCallback(this::onClampCallback);
        this.pixelPouchComponent.addOnReleaseCallback(this::onReleaseCallback);
        this.pixelPouchComponent.initializeServo();

        this.liftComponent = new LiftComponent(super.hardwareMap);
        this.swingArmComponent = new SwingArmComponent(super.hardwareMap);

        this.suspensionLiftComponent = new SuspensionLiftComponent(super.hardwareMap);
        this.suspensionLiftComponent.init();

        this.airplaneLauncher = new AirplaneLauncher(super.hardwareMap);
        this.airplaneLauncher.initializeServo();

        this.liftMultiComponentSystem = new LiftMultiComponentSystem(this.liftComponent, this.swingArmComponent, this.pixelPouchComponent);

        this.offsetHeading = changeHeading;
        changeHeading = 0;
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        Initialize();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        long currentTime, previousTime = System.currentTimeMillis();

        while (opModeIsActive()) {

            // Calculate delta-time
            currentTime = System.currentTimeMillis();
            this.deltaTime = (currentTime - previousTime) / 1000.0f;
            previousTime = currentTime;

            this.handleDrive();

            this.pixelPouchComponent.update();
            this.swingArmComponent.update();

            this.liftMultiComponentSystem.update();
            this.suspensionLiftComponent.update();

            this.liftMultiComponentSystem.telemetry(telemetry);

            LiftMultiComponentSystem.State currentLiftState = this.liftMultiComponentSystem.getCurrentState();

            this.pixelPouchComponent.setShouldDetect(currentLiftState == LiftMultiComponentSystem.State.AT_INTAKE_POSITION);

            if(currentLiftState == LiftMultiComponentSystem.State.AT_SCORING_POSITION) {
                this.didAutoChangeSlowMode = true;
                this.isSlowModeActive = true;
            } else if(this.didAutoChangeSlowMode) {
                this.isSlowModeActive = false;
                this.didAutoChangeSlowMode = false;
            }

            double manualPower = gamepad2.right_trigger- gamepad2.left_trigger;
            suspensionLiftComponent.setManualPower(manualPower);

            driverGamePad.update();
            operatorGamePad.update();

            telemetry.update();

        }

    }

    private boolean isDriverPressingB = false;

    private void OnDriverGamePadChange(FtcGamePad gamepad, int button, boolean isPressed) {

        switch (button) {

            case FtcGamePad.GAMEPAD_BACK -> {
                if (isPressed)
                    offsetHeading = drive.getRawExternalHeading();
            }

            case FtcGamePad.GAMEPAD_RBUMPER -> {
                if (isPressed)
                    isSlowModeActive = !isSlowModeActive;
            }

            case FtcGamePad.GAMEPAD_B -> isDriverPressingB = isPressed;

        }

    }

    private boolean isOperatorRightBumper = false;
    private boolean isOperatorLeftBumper = false;

    private void OnOperatorGamePadChange(FtcGamePad gamepad, int button, boolean isPressed) {

        if(shouldUseMasterController) // Forward Controls to Driver
            this.OnDriverGamePadChange(gamepad, button, isPressed);

        switch (button) {

            case FtcGamePad.GAMEPAD_A -> { // Run Intake Forward
                if (isPressed) {
                    intakeComponent.runIntakeForward();
                } else {
                    intakeComponent.stopIntakeMotor();
                }
            }

            case FtcGamePad.GAMEPAD_B -> { // Suspend If Driver is also pressing
                if (isPressed) {
                    if(isDriverPressingB) {
                        suspensionLiftComponent.raiseHooks();
                    }
                }
            }

            case FtcGamePad.GAMEPAD_X -> { // Release Pixel
                if (isPressed)
                    pixelPouchComponent.requestRelease();
            }

            case FtcGamePad.GAMEPAD_DPAD_DOWN -> {
                if (isPressed)
                    liftMultiComponentSystem.goToIntakePosition();
            }

            case FtcGamePad.GAMEPAD_DPAD_LEFT -> {
                if (isPressed) {
                    liftMultiComponentSystem.goToScoringPosition(LiftComponent.LIFT_POSITION_STAGE_LOWER_RELEASE);
                }
            }

            case FtcGamePad.GAMEPAD_DPAD_UP -> {
                if (isPressed) {
                    liftMultiComponentSystem.goToScoringPosition(LiftComponent.LIFT_POSITION_STAGE_MIDDLE_RELEASE);
                }
            }

            case FtcGamePad.GAMEPAD_DPAD_RIGHT -> {
                if (isPressed) {
                    liftMultiComponentSystem.goToScoringPosition(LiftComponent.LIFT_POSITION_STAGE_UPPER_RELEASE);
                }
            }

            case FtcGamePad.GAMEPAD_Y -> {

                if(isPressed) {
                    suspensionLiftComponent.clearPole();
                }

            }

            case FtcGamePad.GAMEPAD_RBUMPER -> isOperatorRightBumper = isPressed;
            case FtcGamePad.GAMEPAD_LBUMPER -> isOperatorLeftBumper = isPressed;

        }

        if(isOperatorRightBumper && isOperatorLeftBumper) {
            this.airplaneLauncher.launch();
        }

    }

    private static final int DRIVER_CONTROLLER_EASE_POW = 1;

    private void handleDrive() {

        double y = Range.clip(Math.pow(-gamepad1.left_stick_y, DRIVER_CONTROLLER_EASE_POW), -1.0, 1.0);
        double xt = (Math.pow(gamepad1.right_trigger, DRIVER_CONTROLLER_EASE_POW) - Math.pow(gamepad1.left_trigger, DRIVER_CONTROLLER_EASE_POW)) * (shouldUseMasterController ? 0 : 1);
        double x = Range.clip(Math.pow(gamepad1.left_stick_x, DRIVER_CONTROLLER_EASE_POW) + xt, -1.0, 1.0);
        double rx = Range.clip(Math.pow(gamepad1.right_stick_x, DRIVER_CONTROLLER_EASE_POW), -1.0, 1.0);

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

        double speedLimit = MAX_ROBOT_SPEED;

        if(isSlowModeActive)
            speedLimit = SLOW_MODE_SPEED;

        frontLeftPower  = Range.clip(frontLeftPower, -speedLimit, speedLimit);
        frontRightPower = Range.clip(frontRightPower, -speedLimit, speedLimit);
        backLeftPower   = Range.clip(backLeftPower, -speedLimit, speedLimit);
        backRightPower  = Range.clip(backRightPower, -speedLimit, speedLimit);

        drive.setMotorPowers(frontLeftPower, backLeftPower, backRightPower, frontRightPower);
    }

    private enum PixelColor {

        YELLOW,
        GREEN,
        WHITE,
        PURPLE

    }

    private PixelColor detectPixelColor() {

        float[] hsvValues = { 0.0f, 0.0f, 0.0f };

        int red = this.pixelPouchComponent.getRed();
        int green = this.pixelPouchComponent.getGreen();
        int blue = this.pixelPouchComponent.getBlue();

        Color.RGBToHSV(red * 8, green * 8, blue * 8, hsvValues);

        float hue = hsvValues[0];

        if(hue < 90)
            return PixelColor.YELLOW;
        else if (hue > 120 && hue < 135) {
            return PixelColor.GREEN;
        } else if(hue > 158 && hue < 166) {
            return PixelColor.WHITE;
        } else if(hue > 200 && hue < 220) {
            return PixelColor.PURPLE;
        } else {
            return PixelColor.WHITE;
        }

    }

    private void onClampCallback() {

        this.triggerPixelClampedRumble();

        switch (detectPixelColor()) {
            case WHITE -> setGamepadStatusColor(1, 1, 0.8);
            case PURPLE -> setGamepadStatusColor(1, 0, 1);
            case YELLOW -> setGamepadStatusColor(1, 0.55, 0);
            case GREEN -> setGamepadStatusColor(0, 1, 0);
        }

    }

    private void onReleaseCallback() {
        this.resetGamepadStatusColors();
    }

    private void triggerPixelClampedRumble() {

        this.gamepad1.rumbleBlips(2);
        this.gamepad2.rumbleBlips(2);

    }

    private void setGamepadStatusColor(double r, double g, double b) {

        this.gamepad1.setLedColor(r, g, b, Gamepad.LED_DURATION_CONTINUOUS);
        this.gamepad2.setLedColor(r, g, b, Gamepad.LED_DURATION_CONTINUOUS);

    }

    private void resetGamepadStatusColors() {

        this.gamepad1.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
        this.gamepad2.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);

    }

    public static void setChangeHeading(double heading) {
        Teleop.changeHeading = heading;
    }

}
