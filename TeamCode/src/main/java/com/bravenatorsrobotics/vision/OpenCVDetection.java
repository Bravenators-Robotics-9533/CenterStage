package com.bravenatorsrobotics.vision;

import com.bravenatorsrobotics.HardwareMapIdentities;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class OpenCVDetection {

    private final OpenCvCamera camera;
    private final TeamPropPipeline pipeline;

    public OpenCVDetection(Telemetry telemetry, HardwareMap hardwareMap) {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, HardwareMapIdentities.WEBCAM);

        this.camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        this.pipeline = new TeamPropPipeline(telemetry);

    }

    public void initializeCamera() {
        this.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                // Start Streaming
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(pipeline);

            }

            @Override
            public void onError(int errorCode) {
                System.err.println("Could not open OpenCVCamera");
            }
        });
    }

}
