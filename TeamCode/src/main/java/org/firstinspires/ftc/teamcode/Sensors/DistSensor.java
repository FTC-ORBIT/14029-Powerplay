package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistSensor {

    private final ModernRoboticsI2cRangeSensor sensor;

    public DistSensor (HardwareMap hardwareMap) {
        sensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distanceSensor");
    }

    public float getDistance(){return (float) sensor.getDistance(DistanceUnit.CM);}

}
