package com.bravenatorsrobotics.concept;

import com.acmerobotics.dashboard.config.Config;
import com.bravenatorsrobotics.vision.OpenCVDetection;
import com.bravenatorsrobotics.vision.TeamPropPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name="Concept Team Prop Identification", group="Concept")
@Config
public class ConceptTeamPropIdentification extends LinearOpMode {

    private enum Position {
        LEFT, RIGHT, CENTER
    }

    private Position position = Position.LEFT;

    public static TeamPropPipeline.DetectionColorPipeline colorPipeline = TeamPropPipeline.DetectionColorPipeline.PIPELINE_RED;

    @Override
    public void runOpMode() throws InterruptedException {

        OpenCVDetection openCVDetection = new OpenCVDetection(hardwareMap);
        openCVDetection.getTeamPropPipeline().setDetectionColorPipeline(colorPipeline);
        openCVDetection.startStreaming();

        waitForStart();

        while(opModeIsActive()) {

            telemetry.addData("Is Detecting", openCVDetection.getTeamPropPipeline().isDetecting());

            if(openCVDetection.getTeamPropPipeline().isDetecting()) {
                telemetry.addData("Position", openCVDetection.getTeamPropPipeline().getDetectedPosition().val[0] + ", " + openCVDetection.getTeamPropPipeline().getDetectedPosition().val[1]);
            }

            if(colorPipeline == TeamPropPipeline.DetectionColorPipeline.PIPELINE_RED) {
                if (!openCVDetection.getTeamPropPipeline().isDetecting() || openCVDetection.getTeamPropPipeline().getDetectedPosition().val[0] <= 300)
                    position = Position.LEFT;
                else if (openCVDetection.getTeamPropPipeline().getDetectedPosition().val[0] <= 850) // Cent
                    position = Position.CENTER;
                else if (openCVDetection.getTeamPropPipeline().getDetectedPosition().val[0] > 850) // Right
                    position = Position.RIGHT;
            } else {
                if (!openCVDetection.getTeamPropPipeline().isDetecting() || openCVDetection.getTeamPropPipeline().getDetectedPosition().val[0] <= 400)
                    position = Position.LEFT;
                else if (openCVDetection.getTeamPropPipeline().getDetectedPosition().val[0] <= 850) // Cent
                    position = Position.CENTER;
                else if (openCVDetection.getTeamPropPipeline().getDetectedPosition().val[0] > 850) // Right
                    position = Position.RIGHT;
            }

            telemetry.addData("Position", position.name());

            telemetry.update();

        }

    }

}
