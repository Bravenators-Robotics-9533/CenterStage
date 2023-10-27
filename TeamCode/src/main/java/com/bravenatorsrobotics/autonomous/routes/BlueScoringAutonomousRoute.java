package com.bravenatorsrobotics.autonomous.routes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.bravenatorsrobotics.autonomous.Auto;
import com.bravenatorsrobotics.autonomous.TeamPropLocation;

import roadrunner.drive.MecanumDrive;
import roadrunner.trajectorysequence.TrajectorySequence;

public class BlueScoringAutonomousRoute extends AutonomousRoute {

    private static final double WIDTH = 16.6;
    private static final double HEIGHT = 17.0;

    private static final Pose2d START_POSITION = new Pose2d(12, 70 - WIDTH / 2.0, Math.toRadians(0));

    private TrajectorySequence leftFirstSequence;
    private TrajectorySequence rightFirstSequence;
    private TrajectorySequence centerFirstSequence;

    public BlueScoringAutonomousRoute(Auto auto, MecanumDrive drive) {
        super(auto, drive);
    }

    @Override
    public void initialize() {

        this.leftFirstSequence = drive.trajectorySequenceBuilder(START_POSITION)
                .lineTo(new Vector2d(12, 32))
                .lineToSplineHeading(new Pose2d(15.0, 26, Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(12, 26))
                .lineTo(new Vector2d(12, 40))
            .build();

        this.rightFirstSequence = drive.trajectorySequenceBuilder(START_POSITION)
                .lineTo(new Vector2d(12, 32))
                .lineToSplineHeading(new Pose2d(6.0, 28, Math.toRadians(-90)))
                .addTemporalMarker(() -> auto.pixelFunnelComponent.releasePixel())
                .waitSeconds(0.25)
                .lineToLinearHeading(new Pose2d(12, 40, Math.toRadians(0)))
            .build();

        this.centerFirstSequence = drive.trajectorySequenceBuilder(START_POSITION)
                .lineTo(new Vector2d(12, 32))
                .addTemporalMarker(() -> auto.pixelFunnelComponent.releasePixel())
                .waitSeconds(0.25)
                .lineTo(new Vector2d(12, 40))
            .build();

    }

    @Override
    public void run(TeamPropLocation teamPropLocation) {

    }

}
