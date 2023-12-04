package com.bravenatorsrobotics.autonomous.routes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.bravenatorsrobotics.autonomous.Auto;
import com.bravenatorsrobotics.autonomous.TeamPropLocation;
import com.bravenatorsrobotics.components.LiftComponent;

import roadrunner.drive.MecanumDrive;
import roadrunner.trajectorysequence.TrajectorySequence;

public class RedScoringAutonomousRoute extends AutonomousRoute {

    private static final double WIDTH = 16.6;
    private static final double HEIGHT = 17.0;

    private static final Pose2d START_POSITION = new Pose2d(12, -70 + WIDTH / 2.0, Math.toRadians(180));

    private TrajectorySequence leftFirstSequence;
    private TrajectorySequence centerFirstSequence;
    private TrajectorySequence rightFirstSequence;

    private TrajectorySequence leftLineUpBoard;
    private TrajectorySequence centerLineUpBoard;
    private TrajectorySequence rightLineUpBoard;

    private double DRIVE_TO_BOARD_TO_SCORE_DIST = 7;
    private double WAIT_TIME_FOR_LIFT = 4.0;

    public RedScoringAutonomousRoute(Auto auto, MecanumDrive drive) {
        super(auto, drive);
    }

    @Override
    public void initialize() {

        this.leftFirstSequence = drive.trajectorySequenceBuilder(START_POSITION)
                .lineTo(new Vector2d(12, -32))
                .lineToSplineHeading(new Pose2d(6.0, -28, Math.toRadians(-90)))
                .addTemporalMarker(() -> auto.pixelFunnelComponent.releasePixel())
                .waitSeconds(0.25)
                .lineTo(new Vector2d(12, -28))
            .build();

        this.leftLineUpBoard = drive.trajectorySequenceBuilder(this.leftFirstSequence.end())
                .addTemporalMarker(() -> auto.liftMultiComponentSystem.goToScoringPosition(LiftComponent.LIFT_POSITION_STAGE_MIDDLE_RELEASE))
                .waitSeconds(WAIT_TIME_FOR_LIFT)
                .lineToLinearHeading(new Pose2d(53.5, -23.5, Math.toRadians(180)))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> auto.pixelPouchComponent.requestRelease())
                .waitSeconds(0.25)
                .forward(DRIVE_TO_BOARD_TO_SCORE_DIST)
                .addTemporalMarker(() -> auto.liftMultiComponentSystem.goToIntakePosition())
                .strafeLeft( 36)
                .back(14)
            .build();

        this.centerFirstSequence = drive.trajectorySequenceBuilder(START_POSITION)
                .lineTo(new Vector2d(14, -24))
                .addTemporalMarker(() -> auto.pixelFunnelComponent.releasePixel())
                .waitSeconds(0.3)
                .lineTo(new Vector2d(12, -32))
            .build();

        this.centerLineUpBoard = drive.trajectorySequenceBuilder(this.centerFirstSequence.end())
                .addTemporalMarker(() -> auto.liftMultiComponentSystem.goToScoringPosition(LiftComponent.LIFT_POSITION_STAGE_MIDDLE_RELEASE))
                .waitSeconds(WAIT_TIME_FOR_LIFT)
                .lineToLinearHeading(new Pose2d(49, -32, Math.toRadians(180)))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> auto.pixelPouchComponent.requestRelease())
                .waitSeconds(0.25)
                .forward(DRIVE_TO_BOARD_TO_SCORE_DIST)
                .addTemporalMarker(() -> auto.liftMultiComponentSystem.goToIntakePosition())
                .strafeLeft(29)
                .back(14)
            .build();

        this.rightFirstSequence = drive.trajectorySequenceBuilder(START_POSITION)
                .lineTo(new Vector2d(24, -34))
                .addTemporalMarker(() -> auto.pixelFunnelComponent.releasePixel())
                .waitSeconds(0.25)
                .lineTo(new Vector2d(24, -40))
                .splineToConstantHeading(new Vector2d(12, -48), Math.toRadians(180))
            .build();

        this.rightLineUpBoard = drive.trajectorySequenceBuilder(this.rightFirstSequence.end())
                .addTemporalMarker(() -> auto.liftMultiComponentSystem.goToScoringPosition(LiftComponent.LIFT_POSITION_STAGE_MIDDLE_RELEASE))
                .waitSeconds(WAIT_TIME_FOR_LIFT)
                .lineToLinearHeading(new Pose2d(50, -33, Math.toRadians(180)))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> auto.pixelPouchComponent.requestRelease())
                .waitSeconds(0.25)
                .forward(DRIVE_TO_BOARD_TO_SCORE_DIST)
                .addTemporalMarker(() -> auto.liftMultiComponentSystem.goToIntakePosition())
                .strafeLeft(19.5)
                .back(14)
            .build();
    }

    private void idle() {
        while(opModeIsActive() && drive.isBusy()) {
            auto.pixelPouchComponent.update();
            auto.swingArmComponent.update();

            auto.liftMultiComponentSystem.update();
            auto.liftMultiComponentSystem.telemetry(auto.telemetry);

            drive.update();
        }
    }

    @Override
    public void run(TeamPropLocation teamPropLocation) {

        TrajectorySequence firstTrajectorySequence;

        switch (teamPropLocation) {
            case LEFT: firstTrajectorySequence = this.leftFirstSequence; break;
            case RIGHT: firstTrajectorySequence = this.rightFirstSequence; break;

            default:
            case CENTER: firstTrajectorySequence = this.centerFirstSequence; break;
        }

        drive.setPoseEstimate(START_POSITION);
        drive.followTrajectorySequenceAsync(firstTrajectorySequence);

        this.idle();

        TrajectorySequence secondTrajectorySequence;

        switch (teamPropLocation) {
            case LEFT: secondTrajectorySequence = this.leftLineUpBoard; break;
            case RIGHT: secondTrajectorySequence = this.rightLineUpBoard; break;

            default:
            case CENTER: secondTrajectorySequence = this.centerLineUpBoard; break;
        }

        drive.setPoseEstimate(firstTrajectorySequence.end());
        drive.followTrajectorySequenceAsync(secondTrajectorySequence);

        this.idle();

    }
}
