package com.bravenatorsrobotics.autonomous;

import com.bravenatorsrobotics.autonomous.routes.AutonomousRoute;
import com.bravenatorsrobotics.autonomous.routes.BlueScoringAutonomousRoute;
import com.bravenatorsrobotics.autonomous.routes.RedScoringAutonomousRoute;
import com.bravenatorsrobotics.components.LiftComponent;
import com.bravenatorsrobotics.components.PixelFunnelComponent;
import com.bravenatorsrobotics.components.PixelPouchComponent;
import com.bravenatorsrobotics.components.SwingArmComponent;
import com.bravenatorsrobotics.config.Config;
import com.bravenatorsrobotics.multiComponentSystem.LiftMultiComponentSystem;
import com.bravenatorsrobotics.vision.OpenCVDetection;
import com.bravenatorsrobotics.vision.TeamPropPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import roadrunner.drive.MecanumDrive;

@Autonomous(name="Autonomous", group="Competition")
public class Auto extends LinearOpMode {

    public PixelFunnelComponent pixelFunnelComponent;

    public LiftComponent liftComponent;
    public SwingArmComponent swingArmComponent;
    public PixelPouchComponent pixelPouchComponent;

    public LiftMultiComponentSystem liftMultiComponentSystem;

    private TeamPropLocation teamPropLocation = TeamPropLocation.LEFT;

    private Config config;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialing...");
        telemetry.update();

        // Config
        this.config = new Config(hardwareMap.appContext);

        // Setup Drive System
        MecanumDrive drive = new MecanumDrive(this.hardwareMap);

        // Setup Route
        AutonomousRoute route = config.GetStartingPosition() == Config.StartingPosition.RED ?
                new RedScoringAutonomousRoute(this, drive) :
                new BlueScoringAutonomousRoute(this, drive);
        route.initialize();

        // Setup OpenCV Team Prop Identification
//        OpenCVDetection openCVDetection = new OpenCVDetection(this.hardwareMap);
//        openCVDetection.getTeamPropPipeline().setDetectionColorPipeline(
//                config.GetStartingPosition() == Config.StartingPosition.RED ?
//                TeamPropPipeline.DetectionColorPipeline.PIPELINE_RED :
//                TeamPropPipeline.DetectionColorPipeline.PIPELINE_BLUE);
//        openCVDetection.startStreaming();

        // Pixel Funnel
        this.pixelFunnelComponent = new PixelFunnelComponent(this.hardwareMap);
        this.pixelFunnelComponent.capturePixel();

        // Lift
        this.liftComponent = new LiftComponent(this.hardwareMap);

        // Swing Arm
        this.swingArmComponent = new SwingArmComponent(this.hardwareMap);

        // Pixel Pouch
        this.pixelPouchComponent = new PixelPouchComponent(this.hardwareMap, false);
        this.pixelPouchComponent.initializeServo();
        this.pixelPouchComponent.requestClose();
        this.pixelPouchComponent.setClampPosition(PixelPouchComponent.CLAMP_CLOSE_POSITION);

        // Lift Multi Component System
        this.liftMultiComponentSystem = new LiftMultiComponentSystem(this.liftComponent, this.swingArmComponent, this.pixelPouchComponent);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Detect Team Prop
//        syncDetectTeamProp(openCVDetection); // LOCKS THREAD!!!

        // Stop Streaming
//        openCVDetection.stopStreaming();

        waitForStart(); // For Safety wait again for start // LOCKS THREAD!!!

        // Set to intake position
        this.swingArmComponent.goToIntakePosition();

        route.run(this.teamPropLocation);

    }

    private void syncDetectTeamProp(OpenCVDetection openCVDetection) {

        while(!isStarted()) {

            // Compute team prop location
            if(this.config.GetStartingPosition() == Config.StartingPosition.RED) {
                if(!openCVDetection.getTeamPropPipeline().isDetecting() || openCVDetection.getTeamPropPipeline().getDetectedPosition().val[0] <= 300)
                    this.teamPropLocation = TeamPropLocation.LEFT;
                else if(openCVDetection.getTeamPropPipeline().getDetectedPosition().val[0] <= 850)
                    this.teamPropLocation = TeamPropLocation.CENTER;
                else if(openCVDetection.getTeamPropPipeline().getDetectedPosition().val[0] > 850)
                    this.teamPropLocation = TeamPropLocation.RIGHT;
            } else {
                if(!openCVDetection.getTeamPropPipeline().isDetecting() || openCVDetection.getTeamPropPipeline().getDetectedPosition().val[0] <= 400)
                    this.teamPropLocation = TeamPropLocation.LEFT;
                else if(openCVDetection.getTeamPropPipeline().getDetectedPosition().val[0] <= 850)
                    this.teamPropLocation = TeamPropLocation.CENTER;
                else if(openCVDetection.getTeamPropPipeline().getDetectedPosition().val[0] > 850)
                    this.teamPropLocation = TeamPropLocation.RIGHT;
            }

            // Print telemetry data of team prop location
            telemetry.addData("Location", this.teamPropLocation.name());
            telemetry.update();
        }

    }

}
