package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistSensor {

    static ModernRoboticsI2cRangeSensor sensor;

    public static void init(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distanceSensor");
    }

    public static float getDistance(){return (float) sensor.getDistance(DistanceUnit.CM);}

}
