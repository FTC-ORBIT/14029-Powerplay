package org.firstinspires.ftc.teamcode.Sensors;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OrbitUtils.RGB;

public class OrbitColorSensor {



        private final ColorSensor colorSensor;

        public OrbitColorSensor(HardwareMap hardwareMap) {
            colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        }

        private float colorMap(float num) {
            return (255 * num) / 4095;
        }

        public RGB rgb() {
            final RGB rgb = new RGB();
            return new RGB();
        }

    }

