package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OrbitTouchSensor {

    private static RevTouchSensor sensor;

    public void init(HardwareMap hardwareMap, String name){
        sensor = hardwareMap.get(RevTouchSensor.class, name);
    }

    public boolean isTouch(){
        return sensor.isPressed();
    }
}
