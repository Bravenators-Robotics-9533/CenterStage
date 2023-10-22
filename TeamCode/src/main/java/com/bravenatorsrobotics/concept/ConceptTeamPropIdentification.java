package com.bravenatorsrobotics.concept;

import com.bravenatorsrobotics.vision.OpenCVDetection;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Concept Team Prop Identification", group="Concept")
public class ConceptTeamPropIdentification extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        OpenCVDetection openCVDetection = new OpenCVDetection(telemetry, hardwareMap);
        openCVDetection.initializeCamera();

        waitForStart();

        while(opModeIsActive()) {


        }

    }

}
