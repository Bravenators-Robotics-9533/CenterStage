package com.bravenatorsrobotics.autonomous.routes;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.bravenatorsrobotics.autonomous.TeamPropLocation;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import roadrunner.drive.DriveConstants;
import roadrunner.drive.MecanumDrive;
import roadrunner.trajectorysequence.TrajectorySequence;
import roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class RedScoringAutonomousRoute extends AutonomousRoute {

    private static final double WIDTH = 16.6;
    private static final double HEIGHT = 17.0;

    private static final Pose2d START_POSITION = new Pose2d(12, -70 + WIDTH / 2.0, Math.toRadians(180));

    private TrajectorySequence sequence;

    public RedScoringAutonomousRoute(LinearOpMode opMode, MecanumDrive drive) {
        super(opMode, drive);
    }

    @Override
    public void initialize() {

        sequence = drive.trajectorySequenceBuilder(START_POSITION)
            .lineTo(new Vector2d(12, -32))
            .waitSeconds(1)
            .lineToSplineHeading(new Pose2d(WIDTH / 2, -30, Math.toRadians(-90)))
            // RELEASE THE PIXEL
//            .lineToLinearHeading(new Pose2d(40, -36, 0))
//            .lineTo(new Vector2d(30, -36))
//            .splineToSplineHeading(new Pose2d(60, -65 + WIDTH / 2.0, Math.toRadians(180)), Math.toRadians(400))
            .build();

    }

    @Override
    public void run(TeamPropLocation teamPropLocation) {

        drive.setPoseEstimate(START_POSITION);
        drive.followTrajectorySequence(sequence);
//
//        while(opModeIsActive()) {
//            drive.update();
//        }

    }
}
