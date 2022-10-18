package org.firstinspires.ftc.teamcode.Sensors;
import android.media.UnsupportedSchemeException;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.positionTracker.PoseTracker;

public class OrbitColorSensor {



        private static ColorSensor color;
        public static void init(HardwareMap hardwareMap) {
            color = hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "Color");

        }

        private static float colorMap(float num) {
            return (255 * num) / 4095;
        }

        public static float[] rgb() {
            return new float[]{colorMap(color.red()), colorMap(color.green()), colorMap(color.blue())};
        }

        public static String readColor(float red, float green, float blue) {
            if ((red >= rgb()[0] + 10 || red <= rgb()[0] - 10) && (green >= rgb()[1] + 10)) {
                return "red";
            }
            else if( ( green <= rgb()[1] - 10) && (blue >= rgb()[2] + 10 || blue <= rgb()[2] - 10)){
                return "blue";
            }
            else
                return "null";
        }
    }

