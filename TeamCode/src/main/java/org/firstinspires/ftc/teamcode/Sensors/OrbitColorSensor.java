package org.firstinspires.ftc.teamcode.Sensors;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OrbitColorSensor {



        private static ColorSensor color;

        public static void init(HardwareMap hardwareMap) {
            color = hardwareMap.get(ColorSensor.class, "Color");
        }

        private static float colorMap(float num) {
            return (255 * num) / 4095;
        }

        public static float[] rgb() {
            return new float[]{colorMap(color.red()), colorMap(color.green()), colorMap(color.blue())};
        }

        public static boolean readColor(float red, float green, float blue) {

            if ((red >= rgb()[0] + 10 || red <= rgb()[0] - 10) && (green >= rgb()[1] + 10 || green <= rgb()[1] - 10) && (blue >= rgb()[2] + 10 || blue <= rgb()[2] - 10)) {
                return true;
            } else return false;
        }
    }

