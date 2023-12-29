package com.bravenatorsrobotics.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.bravenatorsrobotics.HardwareMapIdentities;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class OpenCVDetection {

    private final OpenCvCamera camera;
    private final TeamPropPipeline pipeline;

    private boolean isStreaming = false;
    private boolean isRequestingImmediateStopStreaming = false;

    public OpenCVDetection(HardwareMap hardwareMap) {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, HardwareMapIdentities.POUCH_WEBCAM);

        this.camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        this.pipeline = new TeamPropPipeline();

        FtcDashboard.getInstance().startCameraStream(this.camera, 30);

    }

    public void startStreaming() {
        this.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                camera.setPipeline(pipeline);
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                isStreaming = true;

                if(isRequestingImmediateStopStreaming) {
                    isRequestingImmediateStopStreaming = false;
                    stopStreaming();
                }

            }

            @Override
            public void onError(int errorCode) {
                System.err.println("Could not open OpenCVCamera");
                isStreaming = false;
            }
        });
    }

    public void stopStreaming() {
        if(this.isStreaming) {
            this.camera.stopStreaming();
            this.isStreaming = false;
        } else {
            this.isRequestingImmediateStopStreaming = true;
        }
    }

    public TeamPropPipeline getTeamPropPipeline() { return this.pipeline; }

}
