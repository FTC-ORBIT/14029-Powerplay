package org.firstinspires.ftc.teamcode.robotSubSystems.claw;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw {
    private  static  ClawState lastState = ClawState.OPEN;
    private static float pos;
    public static CRServo clawServo;
    public static double power = 0;

    public static void init(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(CRServo.class, "clawServo");
        clawServo.setDirection(CRServo.Direction.REVERSE);
    }

    public static void operate(ClawState state) {
        switch (state) {
            case OPEN:
                pos = ClawConstants.open;
                break;
            case CLOSE:
                pos = ClawConstants.closed;
                break;
            case STOP:
                pos = ClawConstants.stop;
        }
        clawServo.setPower(pos);


    }

    public static boolean isClawCorrectPos(float wantedPos) {
        return clawServo.getPower() == wantedPos;
    }

    public static void clawTest (Gamepad gamepad, Telemetry telemetry) {
        clawServo.setPower(gamepad.left_stick_y);
        telemetry.addData("position", clawServo.getPower());
    }
}
