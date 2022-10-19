package org.firstinspires.ftc.teamcode.robotSubSystems.arm;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    private static float pos;
    public static Servo armServo;

    public static void init(HardwareMap hardwareMap){
        armServo = hardwareMap.get(Servo.class, "armServo");
    }

    public static void operate(ArmState state){
        switch (state){
            case FRONT:
                pos = ArmConstants.front;
                break;
            case BACK:
                pos = ArmConstants.back;
                break;
        }
        armServo.setPosition(pos);
    }


    public static void reset(){
        armServo.setPosition(ArmConstants.reset);
    }
}
