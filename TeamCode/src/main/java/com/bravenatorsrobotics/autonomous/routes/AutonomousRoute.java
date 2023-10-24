package com.bravenatorsrobotics.autonomous.routes;

import com.bravenatorsrobotics.autonomous.TeamPropLocation;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import roadrunner.drive.MecanumDrive;

public abstract class AutonomousRoute {

    protected final MecanumDrive drive;

    private final LinearOpMode opMode;
    private final ElapsedTime timer = new ElapsedTime();

    public AutonomousRoute(LinearOpMode opMode, MecanumDrive drive) {
        this.opMode = opMode;
        this.drive = drive;
    }

    public abstract void initialize();
    public abstract void run(TeamPropLocation teamPropLocation);

    protected void waitMillis(int millis) {
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() <= millis);
    }

    protected LinearOpMode getOpMode() { return this.opMode; }
    protected boolean opModeIsActive() { return this.opMode.opModeIsActive(); }

}
