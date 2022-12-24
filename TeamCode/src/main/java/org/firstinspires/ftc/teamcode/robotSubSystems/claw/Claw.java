package org.firstinspires.ftc.teamcode.robotSubSystems.claw;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw {

    private static float pos;
    public static Servo clawServo;

    public static void init(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawServo.setDirection(Servo.Direction.REVERSE);
        clawServo.setPosition(ClawConstants.open);
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
        clawServo.setPosition(gamepad.right_trigger);
        telemetry.addData("position", clawServo.getPosition());
    }
}
