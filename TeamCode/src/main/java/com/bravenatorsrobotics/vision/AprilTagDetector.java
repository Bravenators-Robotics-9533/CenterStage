package com.bravenatorsrobotics.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class AprilTagDetector {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    public void initialize(LinearOpMode opMode, WebcamName webcam) {

        this.aprilTagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(1385.92, 1385.92, 951.982, 534.084)
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        this.aprilTagProcessor.setDecimation(3); // Originally was two

        this.visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(this.aprilTagProcessor)
                .build();

        // Exposure Change

//        if(this.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//
//            while(!opMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
//                sleep(10);
//            }
//
//        }
//
//        if(!opMode.isStopRequested()) {
//            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//            if(exposureControl.getMode() != ExposureControl.Mode.Manual) {
//                exposureControl.setMode(ExposureControl.Mode.Manual);
//                sleep(50);
//            }
//
//            exposureControl.setExposure(6, TimeUnit.MILLISECONDS);
//            sleep(20);
//
//            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
//            gainControl.setGain(250);
//
//            sleep(20);
//        }

    }

    public ArrayList<AprilTagDetection> getDetections() {

        return this.aprilTagProcessor.getDetections();

    }


    private void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }


}
