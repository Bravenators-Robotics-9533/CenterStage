package com.bravenatorsrobotics.vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.tensorflow.lite.support.image.ImageProcessor;

@Config
public class TeamPropPipeline extends OpenCvPipeline {

    public static Scalar lowerBlue = new Scalar(0, 110, 180);
    public static Scalar upperBlue = new Scalar(10, 200, 255);

    private final Telemetry telemetry;

    public TeamPropPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    private Mat preprocessFrame(Mat frame) {

        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        return hsvFrame;

    }

    private Mat deriveBlueFromFrame(Mat processedHSVFrame) {
        Mat blueMask = new Mat();
        Core.inRange(processedHSVFrame, lowerBlue, upperBlue, blueMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_CLOSE, kernel);

        return blueMask;
    }

    @Override
    public Mat processFrame(Mat input) {

        Mat processedFrame = preprocessFrame(input);
        Mat blueMask = deriveBlueFromFrame(processedFrame);

        return blueMask;
    }

}
