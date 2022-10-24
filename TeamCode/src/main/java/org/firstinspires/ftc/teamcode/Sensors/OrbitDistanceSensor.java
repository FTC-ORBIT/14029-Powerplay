package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OrbitDistanceSensor {

    private final DistanceSensor distanceSensor;
    private float lastDistance;

    public OrbitDistanceSensor(HardwareMap hardwareMap) {
      distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor")  ;
    }

    public float getDistance(){
        float distance = (float) distanceSensor.getDistance(DistanceUnit.CM);
        if ( distance < 200) {
            lastDistance = distance;
            return distance;
        } else {
            return lastDistance;
        }
    }

}
