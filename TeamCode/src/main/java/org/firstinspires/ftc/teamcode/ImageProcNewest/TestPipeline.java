package org.firstinspires.ftc.teamcode.ImageProcNewest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;


import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.RotatedRect;
import java.util.ArrayList;
import java.util.List;

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

        RotatedRect[] minRect = new RotatedRect[contours.size()];
        RotatedRect[] minEllipse = new RotatedRect[contours.size()];
        for (int i=0; i< contours.size(); i++){
            minRect[i] = Imgproc.minAreaRect((new MatOfPoint2f(contours.get(i).toArray())));
            minEllipse[i] = new RotatedRect();
            Imgproc.drawContours(input, contours, i, new Scalar(255, 0, 0));

            if(contours.get(i).rows() > 5){
                minEllipse[i] = Imgproc.fitEllipse(new MatOfPoint2f(contours.get(i).toArray()));
                Imgproc.ellipse(input, minEllipse[i], new Scalar(255, 0, 0), 2);
                Point[] rectPoints = new Point[4];
                minRect[i].points(rectPoints);
                for(int j = 0; j < 4; j++){
                    Imgproc.line(input, rectPoints[j], rectPoints[(j+1) % 4], new Scalar(255, 0, 0));
                    Point jPnt = new Point(rectPoints[j].x, rectPoints[j].y-10);
                    Imgproc.putText(input, Integer.toString(j), jPnt,
                            1, 2, new Scalar(255, 0, 0));
                }

            }
        }

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

        }


        //Marking the closest junction:
        if (area > minArea && areaRect.width > minWidth){
//            dist = -11*areaRect.width + 62;
//            //Imgproc.arcLength(, true);
//            packet.put("area", area);
//            packet.put("distance", dist);
//            packet.put("width: ", areaRect.width);
//            dashboard.sendTelemetryPacket(packet);
//            detected = true;
//
//            Imgproc.rectangle(input, areaRect, new Scalar (0, 255, 0));

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



    }
}
