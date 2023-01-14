package org.firstinspires.ftc.teamcode.robotSubSystems.arm;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

    private static float pos;
    public static Servo armServo;
    private static double power = 0;
    private static boolean lastRight;
    private static boolean lastLeft;

    public static void init(HardwareMap hardwareMap) {
        armServo = hardwareMap.get(Servo.class, "armServo");
        armServo.setDirection(Servo.Direction.REVERSE);
    }

    public static void operate(ArmState state) {
        switch (state) {
            case FRONT:
                pos = ArmConstants.front;
                break;
            case BACK:
                pos = ArmConstants.back;
                break;
        }
        armServo.setPosition(pos);
    }

    public static void reset() {
        armServo.setPosition(ArmConstants.back); // TODO or front...
    }

    public static void testArm (Gamepad gamepad, Telemetry telemetry) {
        if (!lastRight && gamepad.right_bumper){
            power = power + 0.005;
        } else if (!lastLeft && gamepad.left_bumper){
            power = power - 0.005;
        }
        armServo.setPosition(power);
        telemetry.addData("position", power);
        lastRight = gamepad.right_bumper;
        lastLeft = gamepad.left_bumper;

    }
}
