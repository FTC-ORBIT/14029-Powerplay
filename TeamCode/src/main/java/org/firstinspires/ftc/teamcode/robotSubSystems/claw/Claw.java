package org.firstinspires.ftc.teamcode.robotSubSystems.claw;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Sensors.OrbitColorSensor;

public class Claw {
    private  static  ClawState lastState = ClawState.OPEN;
    private static float pos;
    private static Servo clawServo;
    private static OrbitColorSensor clawColorSensor;
    private static float colorControlBlueCone = 0;
    private static float colorControlRedCone = 0;
    private static double power = 0;
    private static boolean lastRight = false;
    private static boolean lastLeft = false;


    public static void init(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
//        clawServo.setDirection(Servo.Direction.REVERSE);
        //clawColorSensor = new OrbitColorSensor(hardwareMap, "clawColorSensor");
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

    public static boolean isClawCorrectPos(float wantedPos) {
        return clawServo.getPosition() == wantedPos;
    }

    public static void clawTest (Gamepad gamepad, Telemetry telemetry) {
        if (!lastRight && gamepad.right_bumper){
            power = power + 0.005;
        } else if (!lastLeft && gamepad.left_bumper){
            power = power - 0.005;
        }
        clawServo.setPosition(power);
        telemetry.addData("position", clawServo.getPosition());
        lastRight = gamepad.right_bumper;
        lastLeft = gamepad.left_bumper;
    }
}
