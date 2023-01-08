package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OrbitColorSensor {

    private final ColorSensor colorSensor;

    public OrbitColorSensor(HardwareMap hardwareMap, String name) {
        colorSensor = hardwareMap.get(ColorSensor.class, name);
    }


    public float[] rgb() {
        return new float[] {colorSensor.red(),colorSensor.green(),colorSensor.blue() };
    }

}
