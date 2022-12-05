package org.firstinspires.ftc.teamcode.robotSubSystems.claw;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    private static double pos;
    public static Servo clawServo;

    public static void init(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }

    public static void operate(ClawState state) {
        switch (state) {
            case OPEN:
                pos = ClawConstants.open;
                break;
            case CLOSE:
                pos = ClawConstants.closed;
                break;
        }
        clawServo.setPosition(pos);
    }

    public static boolean isClawCorrectPos(double wantedPos) {
        return clawServo.getPosition() == wantedPos ? true : false;
    }

    public static void test(double pos){
        clawServo.setPosition(pos);
    }
}
