package com.bravenatorsrobotics.autonomous.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.bravenatorsrobotics.HardwareMapIdentities;
import com.bravenatorsrobotics.vision.AprilTagDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.analysis.integration.IterativeLegendreGaussIntegrator;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

import roadrunner.drive.MecanumDrive;
import roadrunner.trajectorysequence.TrajectorySequence;

public class AlignVerticalToBackdropSequence extends AutonomousSequence {

    private final LinearOpMode opMode;
    private final MecanumDrive drive;
    private final double desiredDistanceInches;

    private AprilTagDetector aprilTagDetector;

    public AlignVerticalToBackdropSequence(LinearOpMode opMode, MecanumDrive drive, double desiredDistanceInches) {
        this.opMode = opMode;
        this.drive = drive;
        this.desiredDistanceInches = desiredDistanceInches;
    }

    public void initializeVision() {

        this.aprilTagDetector = new AprilTagDetector();
        this.aprilTagDetector.initialize(this.opMode, this.opMode.hardwareMap.get(WebcamName.class, HardwareMapIdentities.LIFT_WEBCAM));

    }

    public Pose2d runSequenceSync(Pose2d startPos, double yInches) {

        double distance = 0.0;
        ElapsedTime searchingTimeout = new ElapsedTime();

        boolean shouldDrive = false;

        while(opMode.opModeIsActive() && !shouldDrive && searchingTimeout.seconds() < 3.0) {

            ArrayList<AprilTagDetection> detections = this.aprilTagDetector.getDetections();

            if(detections.size() > 0) {

                AprilTagDetection detection = detections.get(0);
                distance = Math.cos(Math.toRadians(detection.ftcPose.bearing)) * detection.ftcPose.range;

                shouldDrive = true;
            }

        }


        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(startPos)
                .lineToConstantHeading(new Vector2d(startPos.getX() + (distance - desiredDistanceInches), startPos.getY() + yInches))
                .build();

        drive.followTrajectorySequenceAsync(trajectory);

        while(opMode.opModeIsActive() && drive.isBusy()) {
            drive.update();
        }

        return trajectory.end();
    }

}
