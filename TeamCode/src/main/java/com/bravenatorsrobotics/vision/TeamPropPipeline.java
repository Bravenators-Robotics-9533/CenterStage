package com.bravenatorsrobotics.vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;
import org.tensorflow.lite.support.image.ImageProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

@Config
public class TeamPropPipeline extends OpenCvPipeline {

    public static double lowerBlueH = 0, lowerBlueS = 95, lowerBlueV = 175;
    public static double upperBlueH = 10, upperBlueS = 208, upperBlueV = 255;

    public static double lowerRedH = 112, lowerRedS = 110, lowerRedV = 110;
    public static double upperRedH = 130, upperRedS = 240, upperRedV = 255;

    private boolean isDetecting = false;
    private Scalar position = new Scalar(0, 0);

    public enum DetectionColorPipeline {
        PIPELINE_RED,
        PIPELINE_BLUE,
        NONE
    }

    private DetectionColorPipeline detectionColorPipeline = DetectionColorPipeline.NONE;

    private Mat hsvFrame = new Mat();

    private Mat preprocessFrame(Mat frame) {

        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        return hsvFrame;

    }

    private Mat blueMask = new Mat();

    private Mat deriveBlueFromFrame(Mat processedHSVFrame) {
        Scalar lowerBlue = new Scalar(lowerBlueH, lowerBlueS, lowerBlueV);
        Scalar upperBlue = new Scalar(upperBlueH, upperBlueS, upperBlueV);

        Core.inRange(processedHSVFrame, lowerBlue, upperBlue, blueMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_CLOSE, kernel);

        return blueMask;
    }

    private Mat redMask = new Mat();

    private Mat deriveRedFromFrame(Mat processedHSVFrame) {
        Scalar lowerRed = new Scalar(lowerRedH, lowerRedS, lowerRedV);
        Scalar upperRed = new Scalar(upperRedH, upperRedS, upperRedV);

        Core.inRange(processedHSVFrame, lowerRed, upperRed, redMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_CLOSE, kernel);

        return redMask;
    }

    private MatOfPoint findLargestContour(ArrayList<MatOfPoint> contours) {

        MatOfPoint largestContour = null;
        double maxArea = 0;

        for(MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);

            if(area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        return largestContour;

    }

    private Mat hierarchy = new Mat();

    @Override
    public Mat processFrame(Mat input) {

        Mat processedFrame = preprocessFrame(input);

        Mat mask;

        switch (this.detectionColorPipeline) {
            case PIPELINE_BLUE:
                mask = deriveBlueFromFrame(processedFrame);
                break;
            case PIPELINE_RED:
                mask = deriveRedFromFrame(processedFrame);
                break;
            case NONE:
            default:
                return input;
        }

        ArrayList<MatOfPoint> contours = new ArrayList<>();

        // Find all collections of pixels that are in the mask
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest contour which is assumed to be the team prop
        MatOfPoint largestContour = findLargestContour(contours);

        if(largestContour != null) {

            // Outline the largest contour with red
            Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);

            // Calculate the Centroid
            Moments moments = Imgproc.moments(largestContour);
            double cx = moments.get_m10() / moments.get_m00();
            double cy = moments.get_m01() / moments.get_m00();

            String label = "(" + (int) cx + ", " + (int) cy + ")";
            Imgproc.putText(input, label, new Point(cx + 10, cy), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            Imgproc.circle(input, new Point(cx, cy), 5, new Scalar(0, 255, 0), -1);

            this.isDetecting = true;
            this.position.val[0] = cx;
            this.position.val[1] = cy;
        } else {
            this.isDetecting = false;
        }

        return input;
    }

    public void setDetectionColorPipeline(DetectionColorPipeline colorPipeline) { this.detectionColorPipeline = colorPipeline; }
    public DetectionColorPipeline getDetectionColorPipeline() { return this.detectionColorPipeline; }

    public boolean isDetecting() { return this.isDetecting; }
    public Scalar getDetectedPosition() { return this.position; }
}
