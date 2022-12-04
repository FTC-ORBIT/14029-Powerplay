package org.firstinspires.ftc.teamcode.ImageProc;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
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


import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.RotatedRect.*;
import org.opencv.core.RotatedRect;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Handler;

@Config
public class TestPipeline extends OpenCvPipeline {
    public TestPipeline(Telemetry telemetry) {

    }
    Telemetry telemetry;
    public double[] junctionDist = new double[2];
    private static FtcDashboard dashboard = FtcDashboard.getInstance();
    private static TelemetryPacket packet = new TelemetryPacket();
    public static int lh = 0;
    public static int ls = 0;
    public static int lv = 0;
    public static int hh = 180;
    public static int hs = 255;
    public static int hv = 255;


    @Override
    public Mat processFrame(Mat input) {
        /*
        Ranging the yellow objects:


        RGB:
        low 170, 126, 30
        high 255, 196, 131

        HSV:
        low 0, 136, 154
        high 36, 214, 255
        36 cm:
        area
        width

        42cm:
        area
        width

         */


        //Filters:
        Mat picInput = new Mat();
        picInput = Imgcodecs.imread("/sdcard/FIRST/42cm" + ".bmp"); //Junction pic from 36 cm
        Mat blur = new Mat();
        Imgproc.medianBlur(input, blur, 3); //  Odd numbers only
        Mat hsv = new Mat();
        Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_RGB2HSV);



        Scalar low = new Scalar(lh, ls, lv);
        Scalar high = new Scalar(hh, hs, hv);
        Mat mask = new Mat();
        Core.inRange(hsv, low, high, mask);
        List <MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        hierarchy.release();


        // Detecting Parameters
        int cx = -1;
        int cy = -1;
        Rect areaRect = null;
        boolean detected = false;
        double area = 0;
        double minArea = 200;         // The minimum pixels area of a junction from maximum +- 100 cm
        double minWidth = 0;
        double dist;



        //Detecting junctions:
        for (MatOfPoint contour : contours){
            area = Imgproc.contourArea(contour);
            areaRect = Imgproc.boundingRect(contour);
            Moments M = Imgproc.moments(contour);
            cx = (int)(M.m10/M.m00);
            cy = (int)(M.m01/M.m00);
            Size rectSize = new Size(areaRect.width, areaRect.height);
            Point pnt = new Point(cx, cy);
            double angle = Math.atan(areaRect.height/areaRect.width);
            List<MatOfPoint> boxContours = new ArrayList<>();
            Imgproc.drawContours(input, boxContours, 0, new Scalar(128, 128, 128), -1);
            RotatedRect rotRect = new RotatedRect(pnt, rectSize, angle);

        }

        Point pnt = new Point(cx, cy);

        //Marking the closest junction:
        if (area > minArea && areaRect.width > minWidth){
            dist = -11*areaRect.width + 62;
            //Imgproc.arcLength(, true);
            packet.put("area", area);
            packet.put("distance", dist);
            packet.put("width: ", areaRect.width);
            dashboard.sendTelemetryPacket(packet);
            detected = true;

            Imgproc.rectangle(input, areaRect, new Scalar (0, 255, 0));

        }



        return input;

    /*
    point

    w:21, dist: 63
    w:23, dst: 56
    w:40, dist: 36
    w: 49, dist: 30
    equation:
    dist = -11*width + 62

     */


//        Mat dilate = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(24, 24));
//        Mat erode = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(12, 12));
//
//        Imgproc.dilate(mask, output, dilate);
//        Imgproc.dilate(mask, output, dilate);
//
//        List<MatOfPoint> contours = new ArrayList<>();
//        Mat hierarchy = new Mat();
//
//        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);
//        if (hierarchy.size().height > 0 && hierarchy.size().width > 0) {
//            // for each contour, display it in blue
//            for (int idx = 0; idx >= 0; idx = (int) hierarchy.get(0, idx)[0]) {
//                Imgproc.drawContours(input, contours, idx, new Scalar(0, 0, 0));
//
//            }
//
//
////        Size size = new Size(3, 3);
////        Mat blur = new Mat();
////        //Imgproc.blur(gray, blur, size);
////        Mat cannyOut = new Mat();
////        Imgproc.Canny(gray, cannyOut, 100, 200);
////
//
//            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//            MatOfPoint2f[] contoursMask = new MatOfPoint2f[contours.size()];
//            Rect[] boundRect = new Rect[contours.size()];
//            Point[] centers = new Point[contours.size()];
//            float[][] radius = new float[contours.size()][1];
//
//            for (int i = 0; i < contours.size(); i++) {
//                contoursMask[i] = new MatOfPoint2f();
//                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursMask[i].toArray()));
//                centers[i] = new Point();
//                //Imgproc.minEnclosingCircle(contoursMask[i], centers[i], radius[i]);
//
//            }
//            Mat drawing = Mat.zeros(mask.size(), CvType.CV_8UC3);
//            List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursMask.length);
//            for (MatOfPoint2f poly : contoursMask) {
//                contoursPolyList.add(new MatOfPoint(poly.toArray()));
//            }
//            for (int i = 0; i < contours.size(); i++) {
//                Scalar color = new Scalar(255, 0, 0);
//                Imgproc.drawContours(drawing, contoursPolyList, i, color);
//                Imgproc.rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2);
//                Imgproc.circle(drawing, centers[i], (int) radius[i][0], color, 2);
//            }
//
//
////        //h: 13-77
//            //s: 85-255
//            //v: 103-255
//
//
//            // Finding the largest junction
//
//
//            //Imgproc.drawMarker(input, new Point (120, 160), new Scalar(255, 255, 255), Imgproc.MARKER_TILTED_CROSS, 20, 2, Imgproc.LINE_8);
//        }

    }
}