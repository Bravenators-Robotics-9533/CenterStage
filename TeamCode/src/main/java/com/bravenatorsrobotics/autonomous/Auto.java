package com.bravenatorsrobotics.autonomous;

import com.bravenatorsrobotics.autonomous.routes.RedScoringAutonomousRoute;
import com.bravenatorsrobotics.components.PixelFunnelComponent;
import com.bravenatorsrobotics.vision.OpenCVDetection;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import roadrunner.drive.MecanumDrive;

@Autonomous(name="Autonomous", group="Competition")
public class Auto extends LinearOpMode {

    public PixelFunnelComponent pixelFunnelComponent;

    private TeamPropLocation teamPropLocation = TeamPropLocation.LEFT;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialing...");
        telemetry.update();

        // Setup Drive System
        MecanumDrive drive = new MecanumDrive(this.hardwareMap);

        // Setup Autonomous Route
        RedScoringAutonomousRoute route = new RedScoringAutonomousRoute(this, drive);
        route.initialize();

        // Setup OpenCV Team Prop Identification
        OpenCVDetection openCVDetection = new OpenCVDetection(this.hardwareMap);
        openCVDetection.startStreaming();

        this.pixelFunnelComponent = new PixelFunnelComponent(this.hardwareMap);
        this.pixelFunnelComponent.capturePixel();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Detect Team Prop
        syncDetectTeamProp(openCVDetection);

        // Stop Streaming
        openCVDetection.stopStreaming();

        waitForStart(); // For Safety wait again for start

        route.run(this.teamPropLocation);

    }

    private void syncDetectTeamProp(OpenCVDetection openCVDetection) {

        while(!isStarted()) {

            // Compute team prop location
            if(!openCVDetection.getTeamPropPipeline().isDetecting() || openCVDetection.getTeamPropPipeline().getDetectedPosition().val[0] <= 300)
                this.teamPropLocation = TeamPropLocation.LEFT;
            else if(openCVDetection.getTeamPropPipeline().getDetectedPosition().val[0] <= 850)
                this.teamPropLocation = TeamPropLocation.CENTER;
            else if(openCVDetection.getTeamPropPipeline().getDetectedPosition().val[0] > 850)
                this.teamPropLocation = TeamPropLocation.RIGHT;

            // Print telemetry data of team prop location
            telemetry.addData("Location", this.teamPropLocation.name());
            telemetry.update();
        }

    }

}
