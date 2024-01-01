package com.bravenatorsrobotics.autonomous.routes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.bravenatorsrobotics.autonomous.Auto;
import com.bravenatorsrobotics.autonomous.TeamPropLocation;
import com.bravenatorsrobotics.autonomous.sequence.AlignVerticalToBackdropSequence;
import com.bravenatorsrobotics.components.LiftComponent;

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

        this.alignVerticalToBackdropSequence = new AlignVerticalToBackdropSequence(super.auto, super.drive, 6);
        this.alignVerticalToBackdropSequence.initializeVision();

        this.moveOffWall = drive.trajectorySequenceBuilder(START_POSITION)
                .addTemporalMarker(() -> auto.liftMultiComponentSystem.goToScoringPosition(LiftComponent.LIFT_POSITION_STAGE_LOWER_RELEASE))
                .lineToLinearHeading(new Pose2d(36, 35, Math.toRadians(180)))
                .build();
    }

    private void initialScoreOnBackdrop(TeamPropLocation teamPropLocation) {

        Pose2d pos = this.runTrajectorySequence(START_POSITION, this.moveOffWall);

        double offset = switch (teamPropLocation) {
            case LEFT -> 10;
            case RIGHT -> -14;
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

    @Override
    public void run(TeamPropLocation teamPropLocation) {

        this.initialScoreOnBackdrop(teamPropLocation);

    }

}
