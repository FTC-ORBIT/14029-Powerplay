package org.firstinspires.ftc.teamcode.Sensors;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.opencv.imgproc.Imgproc;

public class Color_Sensor {
    static final float[] hsv = new float[3];
    static final float[] rgb = new float[3];
    static NormalizedColorSensor colorSensor;
    static public void init(HardwareMap hardwareMap){
        colorSensor = hardwareMap.get(NormalizedColorSensor.class,"color");

    }
    public static float[] HSV(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsv);
        return hsv;
    }
}
