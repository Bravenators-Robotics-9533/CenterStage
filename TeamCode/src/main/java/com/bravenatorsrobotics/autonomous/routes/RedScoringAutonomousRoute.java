package com.bravenatorsrobotics.autonomous.routes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.bravenatorsrobotics.autonomous.Auto;
import com.bravenatorsrobotics.autonomous.TeamPropLocation;
import com.bravenatorsrobotics.autonomous.sequence.AlignVerticalToBackdropSequence;
import com.bravenatorsrobotics.components.LiftComponent;

import roadrunner.drive.MecanumDrive;
import roadrunner.trajectorysequence.TrajectorySequence;

public class RedScoringAutonomousRoute extends AutonomousRoute {

    private static final Pose2d START_POSITION = new Pose2d(12, -70 + WIDTH / 2.0, Math.toRadians(180));

    private AlignVerticalToBackdropSequence alignVerticalToBackdropSequence;

    private TrajectorySequence moveOffWall;

    public RedScoringAutonomousRoute(Auto auto, MecanumDrive drive) {
        super(auto, drive);
    }

    @Override
    public void initialize() {

        this.alignVerticalToBackdropSequence = new AlignVerticalToBackdropSequence(super.auto, super.drive, 7);
        this.alignVerticalToBackdropSequence.initializeVision();

        this.moveOffWall = drive.trajectorySequenceBuilder(START_POSITION)
                .addTemporalMarker(() -> auto.liftMultiComponentSystem.goToScoringPosition(LiftComponent.LIFT_POSITION_STAGE_LOWER_RELEASE))
                .lineTo(new Vector2d(35, -26))
                .build();

    }

    @Override
    public void run(TeamPropLocation teamPropLocation) {

        Pose2d pos = super.runTrajectorySequence(START_POSITION, this.moveOffWall);

        pos = this.alignVerticalToBackdropSequence.runSequenceSync(pos);
        super.auto.pixelPouchComponent.requestRelease();

        TrajectorySequence moveOffBackdrop = drive.trajectorySequenceBuilder(pos)
                .addDisplacementMarker(1, () -> super.auto.liftMultiComponentSystem.goToIntakePosition())
                .lineTo(new Vector2d(35, -26))
                .build();

        super.runTrajectorySequence(pos, moveOffBackdrop);

        while(opModeIsActive());

    }

}
