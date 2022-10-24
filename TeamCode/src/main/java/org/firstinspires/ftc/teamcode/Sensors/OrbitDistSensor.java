package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OrbitDistSensor {

    private DistanceSensor distanceSensor;

    public OrbitDistSensor(HardwareMap hardwareMap) {
      distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor")  ;
    }

    public float getDistance(){return (float) distanceSensor.getDistance(DistanceUnit.CM);}

}
