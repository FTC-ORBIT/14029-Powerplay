package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OrbitColorSensor {

    private final ColorSensor colorSensor;

    public OrbitColorSensor(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    private float colorMap(float num) {
        return (255 * num) / 4095;
    }

    public float[] rgb() {
        return new float[] { colorMap(colorSensor.red()), colorMap(colorSensor.green()), colorMap(colorSensor.blue()) };
    }

}
