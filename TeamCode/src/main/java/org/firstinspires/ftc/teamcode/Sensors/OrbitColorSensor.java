package org.firstinspires.ftc.teamcode.Sensors;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OrbitColorSensor {



        private ColorSensor color;

        public void init(HardwareMap hardwareMap) {
            color = hardwareMap.get(ColorSensor.class, "Color");
        }

        private float colorMap(float num) {
            return (255 * num) / 4095;
        }

        public float[] rgb() {
            return new float[]{colorMap(color.red()), colorMap(color.green()), colorMap(color.blue())};
        }

    }

