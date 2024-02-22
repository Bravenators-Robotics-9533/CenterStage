package com.bravenatorsrobotics.autonomous.routes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.bravenatorsrobotics.autonomous.Auto;
import com.bravenatorsrobotics.autonomous.TeamPropLocation;
import com.qualcomm.robotcore.util.ElapsedTime;

import roadrunner.drive.MecanumDrive;
import roadrunner.trajectorysequence.TrajectorySequence;

public abstract class AutonomousRoute {

    public final MecanumDrive drive;

    public static final double WIDTH = 16.6;
    public static final double HEIGHT = 17.0;

    public final Auto auto;
    private final ElapsedTime timer = new ElapsedTime();

    public AutonomousRoute(Auto auto, MecanumDrive drive) {
        this.auto = auto;
        this.drive = drive;
    }

    public abstract void initialize();
    public abstract void run(TeamPropLocation teamPropLocation);

    protected void waitMillis(int millis) {
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() <= millis) {
            this.idle();
        }
    }

    /**
     *
     * @param startPosition the current position of the robot
     * @param trajectorySequence the sequence to run
     * @return the ending position after the sequence
     */
    public Pose2d runTrajectorySequence(Pose2d startPosition, TrajectorySequence trajectorySequence) {

        this.drive.setPoseEstimate(startPosition);
        this.drive.followTrajectorySequenceAsync(trajectorySequence);

        this.idle();

        return this.drive.getPoseEstimate();

    }

    protected void idle() {

        while(opModeIsActive() && (drive.isBusy() || auto.liftMultiComponentSystem.isBusy())) {

            auto.pixelPouchComponent.update();
            auto.swingArmComponent.update();

            auto.liftMultiComponentSystem.update();
            auto.liftMultiComponentSystem.telemetry(this.auto.telemetry);

            drive.update();

            this.auto.telemetry.update();

        }

    }

    protected boolean opModeIsActive() { return this.auto.opModeIsActive(); }

}
