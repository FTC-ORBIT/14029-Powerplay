package org.firstinspires.ftc.teamcode.ImageProc;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;


import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class TestPipeline extends OpenCvPipeline {
    public TestPipeline(Telemetry telemetry) {

    }
    Telemetry telemetry;

    @Override
    public Mat processFrame(Mat input) {
        // Detecting junction


//        Mat mat = new Mat();
//        mat = Imgcodecs.imread("/sdcard/FIRST/junction_example.jpg");
//        Mat resizd = new Mat();
//        Imgproc.resize(mat, resizd, new Size(320, 240));

        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
        Mat mask = new Mat();
        Core.inRange(hsv, new Scalar(13, 85 ,103), new Scalar(77, 255, 255), mask);


//        Size size = new Size(3, 3);
//        Mat blur = new Mat();
//        //Imgproc.blur(gray, blur, size);
//        Mat cannyOut = new Mat();
//        Imgproc.Canny(gray, cannyOut, 100, 200);
//
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        MatOfPoint2f[] contoursMask = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        Point[] centers = new Point[contours.size()];
        float[][] radius = new float[contours.size()][1];

        for (int i=0; i < contours.size(); i++){
            contoursMask[i] = new MatOfPoint2f();
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursMask[i].toArray()));
            centers[i] = new Point();
            //Imgproc.minEnclosingCircle(contoursMask[i], centers[i], radius[i]);

        }
        Mat drawing = Mat.zeros(mask.size(), CvType.CV_8UC3);
        List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursMask.length);
        for (MatOfPoint2f poly : contoursMask) {
            contoursPolyList.add(new MatOfPoint(poly.toArray()));
        }
        for (int i = 0; i < contours.size(); i++) {
            Scalar color = new Scalar(255, 0, 0);
            Imgproc.drawContours(drawing, contoursPolyList, i, color);
            Imgproc.rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2);
            Imgproc.circle(drawing, centers[i], (int) radius[i][0], color, 2);
        }




//        //h: 13-77
        //s: 85-255
        //v: 103-255



        // Finding the largest junction


        //Imgproc.drawMarker(input, new Point (120, 160), new Scalar(255, 255, 255), Imgproc.MARKER_TILTED_CROSS, 20, 2, Imgproc.LINE_8);

        return input;

    }
}
