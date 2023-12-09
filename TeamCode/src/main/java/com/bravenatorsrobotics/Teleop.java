package com.bravenatorsrobotics;

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
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
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

    public static double LIVE_ADJUST_MULTIPLE_CONSTANT = 1000.0;

    private boolean isSlowModeActive = false;
    private boolean shouldUseMasterController = false;

    private float deltaTime = 0;

    private double offsetHeading = 0;

    private void Initialize() {

        Config config = new Config(this.hardwareMap.appContext);
        shouldUseMasterController = config.IsSingleControllerOverride();

        this.offsetHeading = config.GetStartingPosition() == Config.StartingPosition.RED
                ? Math.toRadians(-90)
                : Math.toRadians(90);

        this.driverGamePad   = new FtcGamePad("Driver", gamepad1, this::OnDriverGamePadChange);
        this.operatorGamePad = new FtcGamePad("Operator", gamepad2, this::OnOperatorGamePadChange);

        this.drive = new MecanumDrive(super.hardwareMap);
        this.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.intakeComponent = new IntakeComponent(super.hardwareMap);

        this.pixelPouchComponent = new PixelPouchComponent(super.hardwareMap, true);
        this.pixelPouchComponent.addOnClampCallback(this::onClampCallback);
        this.pixelPouchComponent.initializeServo();

        this.liftComponent = new LiftComponent(super.hardwareMap);
        this.swingArmComponent = new SwingArmComponent(super.hardwareMap);

        this.suspensionLiftComponent = new SuspensionLiftComponent(super.hardwareMap);
        this.suspensionLiftComponent.initializeServos();

        this.airplaneLauncher = new AirplaneLauncher(super.hardwareMap);
        this.airplaneLauncher.initializeServo();

        this.liftMultiComponentSystem = new LiftMultiComponentSystem(this.liftComponent, this.swingArmComponent, this.pixelPouchComponent);
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
            this.handleLift();

            this.pixelPouchComponent.update();
            this.swingArmComponent.update();

            this.liftMultiComponentSystem.update();
            this.suspensionLiftComponent.update();

            this.liftMultiComponentSystem.telemetry(telemetry);

            driverGamePad.update();
            operatorGamePad.update();

            telemetry.update();

        }

    }

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

        }

    }

    private boolean isRightBumper = false;
    private boolean isLeftBumper = false;

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
                if (isPressed && !this.suspensionLiftComponent.isLimitSensorTriggered()) {
                    this.suspensionLiftComponent.runLockSequence();
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

            case FtcGamePad.GAMEPAD_RBUMPER -> isRightBumper = isPressed;
            case FtcGamePad.GAMEPAD_LBUMPER -> isLeftBumper = isPressed;

            case FtcGamePad.GAMEPAD_BACK -> {
                if (isPressed)
                    suspensionLiftComponent.unlockLiftLocks();
            }
        }

        if(isRightBumper && isLeftBumper)
            this.airplaneLauncher.launch();

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

        double suspensionLiftPower = Range.clip(Math.pow(gamepad2.right_trigger - gamepad2.left_trigger, 3), -1.0, 1.0);
        this.suspensionLiftComponent.setManualPower(suspensionLiftPower);
//        this.liftMultiComponentSystem.liveAdjustLiftHeight((int) (liveAdjust * LIVE_ADJUST_MULTIPLE_CONSTANT * this.deltaTime));

    }

    private void onClampCallback() {

        this.triggerPixelClampedRumble();

        // Get Color of Pixel
        NormalizedRGBA rgba = this.pixelPouchComponent.getPouchSensorActiveColor();
        setGamepadStatusColor(rgba.red * 255, rgba.green * 255, rgba.blue * 255);
    }

    private void triggerPixelClampedRumble() {

        this.gamepad1.rumbleBlips(2);
        this.gamepad2.rumbleBlips(2);

    }

    private static final int COLOR_STATUS_DURATION = 2000;

    private void setGamepadStatusColor(double r, double g, double b) {

        this.gamepad1.setLedColor(r, g, b, COLOR_STATUS_DURATION);
        this.gamepad2.setLedColor(r, g, b, COLOR_STATUS_DURATION);

    }

}
