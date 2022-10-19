package org.firstinspires.ftc.teamcode.ImageProc;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class Pipeline extends OpenCvPipeline {
    Pipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    Telemetry telemetry;
    public static double lowH = 40;
    public static double highH = 175;
    public static double lowS = 130;
    public static double highS = 170;
    public static double lowV = 75;
    public static double highV = 200;
    static String retPos = "Not Found";
    double contor = 0;
    String currentPos = "Not Found";


    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
        Mat mask = new Mat();
        Core.inRange(hsv, new Scalar(lowH, lowS, lowV), new Scalar(highH, highS, highV), mask);
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        hierarchy.release();
        double contourX = -1;
        double maxArea = -1;
        Rect maxAreaRect = null;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            maxArea = Math.max(area, maxArea);
            telemetry.addData("area",area);
            if (area == maxArea && area > 0) {
                maxAreaRect = Imgproc.boundingRect(contour);
                Moments M = Imgproc.moments(contour);
                contourX = M.m10 / M.m00;
                contor = contourX;
            }
        }
        if (maxAreaRect != null) {
            Imgproc.rectangle(input, new Point(maxAreaRect.x, maxAreaRect.y), new Point(maxAreaRect.x + maxAreaRect.width, maxAreaRect.y + maxAreaRect.height), new Scalar(255, 0, 0), 2);
        }


        double width = mask.size().width;
        if (contourX > 0 && contourX <= width/3) {
            currentPos = "Left";
        } else if (contourX > width/3 && contourX <= 2*width/3) {
            currentPos = "Center";
        } else if (contourX > 2*width/3) {
            currentPos = "Right";
        }
        Imgproc.putText(input, currentPos, new Point(0, 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(0, 0, 0), 2);


        telemetry.addData("Pos", mask);
        telemetry.addData("Pixels", contourX);
        retPos = currentPos;
        return input;

    }
    public static String currentPosition() {
        return retPos;
    }
}