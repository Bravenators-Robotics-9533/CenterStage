package com.bravenatorsrobotics.autonomous.routes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.bravenatorsrobotics.Teleop;
import com.bravenatorsrobotics.autonomous.Auto;
import com.bravenatorsrobotics.autonomous.TeamPropLocation;
import com.bravenatorsrobotics.autonomous.sequence.AlignVerticalToBackdropSequence;
import com.bravenatorsrobotics.components.LiftComponent;

import roadrunner.drive.MecanumDrive;
import roadrunner.trajectorysequence.TrajectorySequence;

public class RedScoringAutonomousRoute extends AutonomousRoute {

    private static final Pose2d START_POSITION = new Pose2d(12, -70 + WIDTH / 2.0, Math.toRadians(180));
    private static final Pose2d OFF_BACKDROP_POS = new Pose2d(35, -26, Math.toRadians(180));

    private AlignVerticalToBackdropSequence alignVerticalToBackdropSequence;

    private TrajectorySequence moveOffWall;

    public RedScoringAutonomousRoute(Auto auto, MecanumDrive drive) {
        super(auto, drive);
    }

    @Override
    public void initialize() {

        this.alignVerticalToBackdropSequence = new AlignVerticalToBackdropSequence(super.auto, super.drive, 6);
        this.alignVerticalToBackdropSequence.initializeVision();

        this.moveOffWall = drive.trajectorySequenceBuilder(START_POSITION)
                .addTemporalMarker(() -> auto.liftMultiComponentSystem.goToScoringPosition(LiftComponent.LIFT_POSITION_STAGE_LOWER_RELEASE))
                .lineTo(new Vector2d(35, -26))
                .build();

        Teleop.setChangeHeading(this.drive.getRawExternalHeading() - Math.toRadians(90));


    }

    private static final double EXPECTED_DISTANCE = 22.37;

    private void scorePixel(TeamPropLocation teamPropLocation) {
        // Get starting position
        Pose2d pos = super.runTrajectorySequence(START_POSITION, this.moveOffWall);

        double offset = switch (teamPropLocation) {
            case LEFT -> 10;
            case RIGHT -> -12.2;
            case CENTER -> 0;
        };

        AlignVerticalToBackdropSequence.AlignResult result = this.alignVerticalToBackdropSequence.runSequenceSync(pos, offset);

        super.auto.pixelPouchComponent.requestRelease(); // Release Pixel
        super.auto.pixelPouchComponent.update();

        double distAdjust = EXPECTED_DISTANCE - result.distance;

        Pose2d adjustedPosition = new Pose2d(result.endPos.getX() + distAdjust, result.endPos.getY(), result.endPos.getHeading());

        // TODO: CALCULATE THIS SEQUENCE ON ANOTHER THREAD DURING PREVIOUS RUN SEQUENCE
        // Calculate move to OFF_BACKDROP_POS with lift retraction at displacement of 1
        TrajectorySequence moveOffBackdrop = drive.trajectorySequenceBuilder(adjustedPosition)
                .addDisplacementMarker(1, () -> super.auto.liftMultiComponentSystem.goToIntakePosition())
                .lineTo(OFF_BACKDROP_POS.vec())
                .build();

        // Run the previously calculated sequence
        super.runTrajectorySequence(adjustedPosition, moveOffBackdrop);
    }

    private void placePurplePixel(TeamPropLocation teamPropLocation) {

        if(teamPropLocation == TeamPropLocation.RIGHT) {
            // RIGHT
            TrajectorySequence placePixelSequence = drive.trajectorySequenceBuilder(OFF_BACKDROP_POS)
                    .lineToLinearHeading(new Pose2d(33, -8, Math.toRadians(0)))
                    .lineToConstantHeading(new Vector2d(24.5, -15.5))
                    .addTemporalMarker(() -> auto.pixelFunnelComponent.releasePixel())
                    .waitSeconds(0.1)
                    .splineToConstantHeading(new Vector2d(27, -10), Math.PI * 3 / 2)
                    .lineTo(new Vector2d(40, -10))
                    .lineToLinearHeading(new Pose2d(40, -66, Math.toRadians(180)))
                    .lineToConstantHeading(new Vector2d(68, -66))
                    .build();

            super.runTrajectorySequence(OFF_BACKDROP_POS, placePixelSequence);
        } else if(teamPropLocation == TeamPropLocation.CENTER) {
            TrajectorySequence placePixelSequence = drive.trajectorySequenceBuilder(OFF_BACKDROP_POS)
                    .lineToLinearHeading(new Pose2d(35 - 15 + 3.5, -26 + 9.5, Math.toRadians(270)))
                    .addTemporalMarker(() -> auto.pixelFunnelComponent.releasePixel())
                    .waitSeconds(0.1)
                    .lineToConstantHeading(new Vector2d(35, -26 + 9.5))
                    .lineToLinearHeading(OFF_BACKDROP_POS)
                    .splineToConstantHeading(new Vector2d(OFF_BACKDROP_POS.getX() + 32, OFF_BACKDROP_POS.getY() - 36), Math.toRadians(0))
                    .build();

            super.runTrajectorySequence(OFF_BACKDROP_POS, placePixelSequence);
        } else if(teamPropLocation == TeamPropLocation.LEFT) {
            TrajectorySequence placePixelSequence = drive.trajectorySequenceBuilder(OFF_BACKDROP_POS)
                    .lineToLinearHeading(new Pose2d(4.3, -26 - 2.75, Math.toRadians(270)))
                    .addTemporalMarker(() -> auto.pixelFunnelComponent.releasePixel())
                    .waitSeconds(0.1)
                    .lineToConstantHeading(new Vector2d(35, -26 - 3))
                    .splineToLinearHeading(new Pose2d(OFF_BACKDROP_POS.getX() + 30, OFF_BACKDROP_POS.getY() - 33, Math.toRadians(180)), Math.toRadians(0))
                    .build();

            super.runTrajectorySequence(OFF_BACKDROP_POS, placePixelSequence);
        }



    }

    @Override
    public void run(TeamPropLocation teamPropLocation) {

        // Score the pixel on the backdrop and then move to the OFF_BACKDROP_POS
        this.scorePixel(teamPropLocation);

        this.placePurplePixel(teamPropLocation);

    }

}
