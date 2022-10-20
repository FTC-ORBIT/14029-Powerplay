package org.firstinspires.ftc.teamcode.ImageProc.AprilTag;

import android.media.Image;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;

import org.openftc.easyopencv.OpenCvPipeline;

public class MainPipeline extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        mat = Imgcodecs.imread("/sdcard/FIRST/10.bmp");

        Mat hsv = new Mat();
        Imgproc.cvtColor(mat, hsv, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsv, new Scalar(50, 50 ,70), new Scalar(60, 100, 100), hsv);
        return input;
    }
}
