package com.bravenatorsrobotics.autonomous.routes;

import com.bravenatorsrobotics.autonomous.Auto;
import com.bravenatorsrobotics.autonomous.TeamPropLocation;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import roadrunner.drive.MecanumDrive;

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

    protected boolean opModeIsActive() { return this.auto.opModeIsActive(); }

}
