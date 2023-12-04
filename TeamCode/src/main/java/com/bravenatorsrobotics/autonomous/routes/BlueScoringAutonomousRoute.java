package com.bravenatorsrobotics.autonomous.routes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.bravenatorsrobotics.autonomous.Auto;
import com.bravenatorsrobotics.autonomous.TeamPropLocation;
import com.bravenatorsrobotics.components.LiftComponent;

import roadrunner.drive.MecanumDrive;
import roadrunner.trajectorysequence.TrajectorySequence;

public class BlueScoringAutonomousRoute extends AutonomousRoute {

    private static final double WIDTH = 16.6;
    private static final double HEIGHT = 17.0;

    private static final double WAIT_SECONDS = 4.0;

    private static final Pose2d START_POSITION = new Pose2d(12, 70 - 8, Math.toRadians(0));

    private TrajectorySequence leftFirstSequence;
    private TrajectorySequence rightFirstSequence;
    private TrajectorySequence centerFirstSequence;

    public BlueScoringAutonomousRoute(Auto auto, MecanumDrive drive) {
        super(auto, drive);
    }

    @Override
    public void initialize() {

        this.leftFirstSequence = drive.trajectorySequenceBuilder(START_POSITION)
                .lineTo(new Vector2d(25, 37))
                .addTemporalMarker(() -> auto.pixelFunnelComponent.releasePixel())
                .waitSeconds(0.25)
                .lineTo(new Vector2d(23, 54))
                .addTemporalMarker(() -> auto.liftMultiComponentSystem.goToScoringPosition(LiftComponent.LIFT_POSITION_STAGE_MIDDLE_RELEASE))
                .waitSeconds(WAIT_SECONDS)
                .splineToLinearHeading(new Pose2d(23 + 17, 54 - 18, Math.toRadians(180)), Math.toRadians(180))
                .lineTo(new Vector2d(23 + 30, 54 - 16))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> auto.pixelPouchComponent.requestRelease())
                .waitSeconds(0.5)
                .forward(10)
                .addTemporalMarker(() -> auto.liftMultiComponentSystem.goToIntakePosition())
                .strafeRight(23)
                .back(20)

            .build();

        this.rightFirstSequence = drive.trajectorySequenceBuilder(START_POSITION)
                .lineToLinearHeading(new Pose2d(12, 70 - 8 - 32, Math.toRadians(-90)))
                .strafeRight(7)
                .addTemporalMarker(() -> auto.pixelFunnelComponent.releasePixel())
                .waitSeconds(0.25)
                .strafeLeft(7)
                .addTemporalMarker(() -> auto.liftMultiComponentSystem.goToScoringPosition(LiftComponent.LIFT_POSITION_STAGE_MIDDLE_RELEASE))
                .waitSeconds(WAIT_SECONDS)
                .splineToLinearHeading(new Pose2d(47, 70 - 8 - 32 -11, Math.toRadians(180)), Math.toRadians(0))
                .lineTo(new Vector2d(50.5, 70 - 8 - 32 - 11))
                .addTemporalMarker(() -> auto.pixelPouchComponent.requestRelease())
                .waitSeconds(0.25)
                .forward(10)
                .addTemporalMarker(() -> auto.liftMultiComponentSystem.goToIntakePosition())
                .strafeRight(37)
                .back(26)
            .build();

        this.centerFirstSequence = drive.trajectorySequenceBuilder(START_POSITION)
                .lineTo(new Vector2d(12, 32))
                .splineToLinearHeading(new Pose2d(26, 32 - 12.5, Math.toRadians(270)), Math.toRadians(180)) // Line Up Pixel Drop
                .addTemporalMarker(() -> auto.pixelFunnelComponent.releasePixel())
                .waitSeconds(0.25)
                .lineTo(new Vector2d(32, 32 - 12.5))
                .addTemporalMarker(() -> auto.liftMultiComponentSystem.goToScoringPosition(LiftComponent.LIFT_POSITION_STAGE_MIDDLE_RELEASE))
                .waitSeconds(WAIT_SECONDS)
                .splineToLinearHeading(new Pose2d(35, 32, Math.toRadians(180)), Math.toRadians(180))
                .lineTo(new Vector2d(35 + 18.5, 32)) // Line To Board
                .addTemporalMarker(() -> auto.pixelPouchComponent.requestRelease())
                .waitSeconds(0.25)
                .lineTo(new Vector2d(35 + 10, 25))
                .addTemporalMarker(() -> auto.liftMultiComponentSystem.goToIntakePosition())
                .lineTo(new Vector2d(35, 25))
                .strafeRight(32)
                .back(26)
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

        switch (teamPropLocation) {
            case CENTER:
                drive.followTrajectorySequenceAsync(this.centerFirstSequence);
                break;
            case RIGHT:
                drive.followTrajectorySequenceAsync(this.rightFirstSequence);
                break;
            default:
            case LEFT:
                drive.followTrajectorySequenceAsync(this.leftFirstSequence);
                break;
        }


        idle();

    }

}
