package com.bravenatorsrobotics.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamPropPipeline extends OpenCvPipeline {

    private final Telemetry telemetry;

    private static final int IMAGE_WIDTH = 1280;
    private static final int IMAGE_HEIGHT = 720;

    private static final Scalar RED = new Scalar(255, 0, 0);

    private static final Point REGION_1_TOP_LEFT = new Point(0, 0);
    private static final Point REGION_1_BOTTOM_RIGHT = new Point(IMAGE_WIDTH * 0.3, IMAGE_HEIGHT);

    private Mat destMat = new Mat();

    public TeamPropPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    private void convertInput(Mat input) {
        Core.extractChannel(input, destMat, 0);
    }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.rectangle(input, REGION_1_TOP_LEFT, REGION_1_BOTTOM_RIGHT, new Scalar(255, 0, 0));

        Mat region1 = input.submat(new Rect(REGION_1_TOP_LEFT, REGION_1_BOTTOM_RIGHT));

        convertInput(input);

        int avg = (int) Core.mean(region1).val[0];

        telemetry.addData("RED REGION 1", avg);
        telemetry.update();

        return input;
    }

}
