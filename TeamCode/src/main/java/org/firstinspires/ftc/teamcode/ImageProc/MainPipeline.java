package org.firstinspires.ftc.teamcode.ImageProc;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;


import org.openftc.easyopencv.OpenCvPipeline;

public class MainPipeline extends OpenCvPipeline {
    public MainPipeline(Telemetry telemetry) {

    }
    Telemetry telemetry;

    @Override
    public Mat processFrame(Mat input) {
        // Detecting junction


        Mat mat = new Mat();
        mat = Imgcodecs.imread("/sdcard/FIRST/junction_example.jpg");
        Mat resizd = new Mat();
        Imgproc.resize(mat, resizd, new Size(320, 240));

        Mat hsv = new Mat();

        Imgproc.cvtColor(mat, hsv, Imgproc.COLOR_BGR2HSV);
        Mat mask = new Mat();
        //h: 13-77
        //s: 85-255
        //v: 103-255
        Core.inRange(hsv, new Scalar(13, 85 ,103), new Scalar(77, 255, 255), mask);



        // Finding the largest junction
        int x = mat.cols();
        int y = mat.rows();

        Point midPoint = new Point(x, y);
        Imgproc.drawMarker(mat, new Point (160, 120), new Scalar(255, 255, 255), Imgproc.MARKER_TILTED_CROSS, 20, 2, Imgproc.LINE_8);

        return mat;

    }
}
