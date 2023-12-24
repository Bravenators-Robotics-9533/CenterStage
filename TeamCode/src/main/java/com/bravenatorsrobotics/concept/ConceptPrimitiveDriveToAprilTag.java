package com.bravenatorsrobotics.concept;

import com.acmerobotics.dashboard.config.Config;
import com.bravenatorsrobotics.HardwareMapIdentities;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

import roadrunner.drive.MecanumDrive;

@TeleOp(name = "Concept Primitive Drive to April Tag", group = "concept")
@Config
@Disabled
public class ConceptPrimitiveDriveToAprilTag extends LinearOpMode {

    public static double DESIRED_DISTANCE = 12.0; // in.

    public static int DESIRED_TAG_ID = 4;

    public static double SPEED_GAIN = 0.02;
    public static double TURN_GAIN = 0.01;

    public static double MAX_AUTO_SPEED = 0.5;
    public static double MAX_AUTO_TURN = 0.25;

    private MecanumDrive mecanumDrive;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    @Override
    public void runOpMode() throws InterruptedException {

        boolean isTargetFound = false;
        double drive = 0.0;
        double turn = 0.0;

        initAprilTag();

        this.mecanumDrive = new MecanumDrive(super.hardwareMap);

        // Wait for the driver to press Start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {

            isTargetFound = false;
            this.desiredTag = null;

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            for(AprilTagDetection detection : currentDetections) {

                if(detection.metadata != null) {

                    if(detection.id == DESIRED_TAG_ID) {

                        isTargetFound = true;
                        desiredTag = detection;
                        break;

                    } else {

                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);

                    }

                } else {
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }

            }

            if (isTargetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            if(gamepad1.left_bumper && isTargetFound) {

                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;

                drive = -Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                telemetry.addData("Auto", "Drive %5.2f, Turn %5.2f", drive, turn);
            } else {

                drive = -gamepad1.left_stick_y / 2.0;
                turn = -gamepad1.right_stick_x / 4.0;

                telemetry.addData("Manual","Drive %5.2f, Turn %5.2f", drive, turn);

            }

            telemetry.update();

            moveRobot(drive, turn);
            sleep(10);

        }

    }

    private void moveRobot(double x, double yaw) {

        double leftPower = x - yaw;
        double rightPower = x + yaw;

        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));

        if(max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        this.mecanumDrive.setMotorPowers(leftPower, leftPower, rightPower, rightPower);

    }

    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder().build();

        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(super.hardwareMap.get(WebcamName.class, HardwareMapIdentities.LIFT_WEBCAM))
                .addProcessor(aprilTag)
                .build();

        setManualExposure(6, 250); // Should do

    }

    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
    }

}
