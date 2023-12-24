package com.bravenatorsrobotics.autonomous.routes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.bravenatorsrobotics.autonomous.Auto;
import com.bravenatorsrobotics.autonomous.TeamPropLocation;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import roadrunner.drive.MecanumDrive;
import roadrunner.trajectorysequence.TrajectorySequence;

public abstract class AutonomousRoute {

    protected final MecanumDrive drive;

    protected final Auto auto;
    private final ElapsedTime timer = new ElapsedTime();

    public AutonomousRoute(Auto auto, MecanumDrive drive) {
        this.auto = auto;
        this.drive = drive;
    }

    public abstract void initialize();
    public abstract void run(TeamPropLocation teamPropLocation);

    protected void waitMillis(int millis) {
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() <= millis);
    }

    /**
     *
     * @param startPosition the current position of the robot
     * @param trajectorySequence the sequence to run
     * @return the ending position after the sequence
     */
    protected Pose2d runTrajectorySequence(Pose2d startPosition, TrajectorySequence trajectorySequence) {

        this.drive.setPoseEstimate(startPosition);
        this.drive.followTrajectorySequenceAsync(trajectorySequence);

        this.idleWhileDriveIsBusy();

        return this.drive.getPoseEstimate();

    }

    private void idleWhileDriveIsBusy() {

        while(opModeIsActive() && drive.isBusy()) {

            auto.pixelPouchComponent.update();
            auto.swingArmComponent.update();

            auto.liftMultiComponentSystem.update();

            drive.update();

        }

    }

    protected boolean opModeIsActive() { return this.auto.opModeIsActive(); }

}
