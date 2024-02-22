package com.bravenatorsrobotics.autonomous.routes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.bravenatorsrobotics.HardwareMapIdentities;
import com.bravenatorsrobotics.Teleop;
import com.bravenatorsrobotics.autonomous.Auto;
import com.bravenatorsrobotics.autonomous.TeamPropLocation;
import com.bravenatorsrobotics.autonomous.sequence.AlignVerticalToBackdropSequence;
import com.bravenatorsrobotics.components.LiftComponent;
import com.bravenatorsrobotics.components.PixelPouchComponent;

import java.security.spec.PSSParameterSpec;

import roadrunner.drive.MecanumDrive;
import roadrunner.trajectorysequence.TrajectorySequence;

public class BlueScoringAutonomousRoute extends AutonomousRoute {

    private static final Pose2d START_POSITION = new Pose2d(12, 72, Math.toRadians(0));
    private static final Pose2d HOME_POSITION = new Pose2d(36, 35, Math.toRadians(180));

    private AlignVerticalToBackdropSequence alignVerticalToBackdropSequence;
    private TrajectorySequence moveOffWall;

    public BlueScoringAutonomousRoute(Auto auto, MecanumDrive drive) {
        super(auto, drive);
    }

    @Override
    public void initialize() {

        this.alignVerticalToBackdropSequence = new AlignVerticalToBackdropSequence(super.auto, this, 6);
        this.alignVerticalToBackdropSequence.initializeVision();

        this.moveOffWall = drive.trajectorySequenceBuilder(START_POSITION)
                .addTemporalMarker(() -> auto.liftMultiComponentSystem.goToScoringPosition(LiftComponent.LIFT_POSITION_STAGE_LOWER_RELEASE))
                .lineToLinearHeading(new Pose2d(36, 35, Math.toRadians(180)))
                .build();

        Teleop.setChangeHeading(this.drive.getRawExternalHeading() - Math.toRadians(90));
    }

    private void initialScoreOnBackdrop(TeamPropLocation teamPropLocation) {

        Pose2d pos = this.runTrajectorySequence(START_POSITION, this.moveOffWall);

        double offset = switch (teamPropLocation) {
            case LEFT -> 10;
            case RIGHT -> -10;
            case CENTER -> 0;
        };

        AlignVerticalToBackdropSequence.AlignResult result = this.alignVerticalToBackdropSequence.runSequenceSync(pos, offset);

        super.auto.pixelPouchComponent.requestRelease();
        super.auto.pixelPouchComponent.update();

        Pose2d adjustedPosition = new Pose2d(54 - this.alignVerticalToBackdropSequence.getDesiredDistanceInches(), result.endPos.getY(), result.endPos.getHeading());

        TrajectorySequence goHome = drive.trajectorySequenceBuilder(adjustedPosition)
                .addDisplacementMarker(1, () -> super.auto.liftMultiComponentSystem.goToIntakePosition())
                .lineTo(HOME_POSITION.vec())
                .build();

        super.runTrajectorySequence(adjustedPosition, goHome);

    }

    private void scorePurplePixel(TeamPropLocation teamPropLocation) {

        TrajectorySequence sequence = switch (teamPropLocation) {
            case LEFT -> drive.trajectorySequenceBuilder(HOME_POSITION)
                    .lineToLinearHeading(new Pose2d(HOME_POSITION.getX() - 12, HOME_POSITION.getY() + 19, Math.toRadians(0)))
                    .addTemporalMarker(() -> auto.pixelFunnelComponent.releasePixel())
                    .waitSeconds(0.1)
                    .lineToConstantHeading(new Vector2d(HOME_POSITION.getX() - 12, HOME_POSITION.getY() + 37))
                    .lineToLinearHeading(new Pose2d(62, HOME_POSITION.getY() + 37, Math.toRadians(0)))
                    .build();
            case CENTER -> drive.trajectorySequenceBuilder(HOME_POSITION)
                    .lineToLinearHeading(new Pose2d(HOME_POSITION.getX() - 15, HOME_POSITION.getY() - 3, Math.toRadians(-90)))
                    .addTemporalMarker(() -> auto.pixelFunnelComponent.releasePixel())
                    .waitSeconds(0.1)
                    .lineToConstantHeading(new Vector2d(HOME_POSITION.getX() - 5, HOME_POSITION.getY() - 4))
                    .splineToLinearHeading(new Pose2d(66, HOME_POSITION.getY() + 38.5, Math.toRadians(0)), Math.toRadians(0))
                    .build();

            case RIGHT -> drive.trajectorySequenceBuilder(HOME_POSITION)
                    .lineToLinearHeading(new Pose2d(HOME_POSITION.getX() - 32, HOME_POSITION.getY() + 14, Math.toRadians(-90)))
                    .addTemporalMarker(() -> auto.pixelFunnelComponent.releasePixel())
                    .waitSeconds(0.1)
                    .lineToConstantHeading(new Vector2d(HOME_POSITION.getX() - 25, HOME_POSITION.getY() + 16))
                    .splineToLinearHeading(new Pose2d(66, HOME_POSITION.getY() + 42, Math.toRadians(0)), Math.toRadians(0))
                    .build();
        };

        super.runTrajectorySequence(HOME_POSITION, sequence);

    }

    @Override
    public void run(TeamPropLocation teamPropLocation) {

        this.initialScoreOnBackdrop(teamPropLocation);
        this.scorePurplePixel(teamPropLocation);

    }

}
