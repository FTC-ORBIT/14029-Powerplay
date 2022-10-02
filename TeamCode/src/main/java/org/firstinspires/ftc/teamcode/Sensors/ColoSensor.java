package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ColoSensor {

    private static ColorSensor color;

    public static void  init (HardwareMap hardwareMap) {
        color = hardwareMap.get(ColorSensor.class, "Color");
    }

    private static float colorMap (float num){
        return (255*num)/4095;
    }
    public static float[] rgb(){
        return new float[]{colorMap(color.red()),colorMap(color.green()), colorMap(color.blue())};
    }
    }

