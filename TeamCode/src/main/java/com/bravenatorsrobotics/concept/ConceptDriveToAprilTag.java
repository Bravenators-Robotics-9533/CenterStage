package com.bravenatorsrobotics.concept;

import android.graphics.drawable.GradientDrawable;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.bravenatorsrobotics.HardwareMapIdentities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

import roadrunner.drive.MecanumDrive;

@TeleOp(name = "Concept Drive to April Tag", group = "concept")
@Config
public class ConceptDriveToAprilTag extends LinearOpMode {

    public static double DESIRED_DISTANCE = 12.0; // in.

    public static int DESIRED_TAG_ID = 4;

    public static double V_MOD = 0.025;
    public static double H_MOD = 0.05;
    public static double R_MOD = 0.25;

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

            boolean shouldManDrive = true;

            if (isTargetFound) {
                Orientation rot = Orientation.getOrientation(desiredTag.rawPose.R, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                double range = desiredTag.ftcPose.range;
                double dist = Math.cos(Math.toRadians(desiredTag.ftcPose.bearing)) * desiredTag.ftcPose.range;
                double skew = Math.sqrt(range * range - dist * dist) * (desiredTag.ftcPose.bearing > 0 ? 1 : -1);

                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);

                telemetry.addData("Range",  "%5.1f inches", range);
                telemetry.addData("Distance",  "%5.1f inches", dist);
                telemetry.addData("Skew",  "%5.1f inches", skew);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);

                // MOVE
                if(gamepad1.left_bumper) {

                    dist -= 10;

                    mecanumDrive.drive((-dist) * V_MOD, -skew * H_MOD, desiredTag.ftcPose.yaw * R_MOD);

                    shouldManDrive = false;

                }
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            if(shouldManDrive) {
                driveRobot();
            }


            telemetry.update();

        }

    }



    private static final double DRIVER_CONTROLLER_EASE_POW = 1;

    private void driveRobot() {

        double y = Range.clip(Math.pow(-gamepad1.left_stick_y, DRIVER_CONTROLLER_EASE_POW), -1.0, 1.0);
        double xt = (Math.pow(gamepad1.right_trigger, DRIVER_CONTROLLER_EASE_POW) - Math.pow(gamepad1.left_trigger, DRIVER_CONTROLLER_EASE_POW));
        double x = Range.clip(Math.pow(gamepad1.left_stick_x, DRIVER_CONTROLLER_EASE_POW) + xt, -1.0, 1.0);
        double rx = Range.clip(Math.pow(gamepad1.right_stick_x, DRIVER_CONTROLLER_EASE_POW), -1.0, 1.0);

        // Read inverse IMU heading, as the UMG heading is CW positive
        double botHeading = 0;

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


        this.mecanumDrive.setMotorPowers(frontLeftPower, backLeftPower, backRightPower, frontRightPower);

    }

    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(1385.92, 1385.92, 951.982, 534.084)
                .build();

        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(super.hardwareMap.get(WebcamName.class, HardwareMapIdentities.LIFT_WEBCAM))
                .setCameraResolution(new Size(1920, 1080))
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
