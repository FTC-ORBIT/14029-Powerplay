package org.firstinspires.ftc.teamcode.ImageProcNewest;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Scalar;
import org.opencv.core.Point;

@Config
public class CapturingPipeline extends OpenCvPipeline {
    CapturingPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    Telemetry telemetry;
    int picCount = 0;


    @Override
    public Mat processFrame(Mat inFrame) {

        picCount = picCount+1;
        String picFileName;
        String picFilePath;

        picFileName= Integer.toString(picCount) + ".bmp";
        picFilePath = "/sdcard/FIRST/" + picFileName;
        Imgcodecs.imwrite(picFilePath, inFrame);
        Imgproc.putText(inFrame, picFileName, new Point(20, 20), 2, 1, new Scalar(255, 0 ,0));

        return inFrame;

    }



}