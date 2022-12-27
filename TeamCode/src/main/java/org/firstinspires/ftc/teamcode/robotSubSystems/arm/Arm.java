package org.firstinspires.ftc.teamcode.robotSubSystems.arm;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

    private static float pos;
    public static CRServo armServo;

    public static void init(HardwareMap hardwareMap) {
        armServo = hardwareMap.get(CRServo.class, "armServo");
        armServo.setDirection(CRServo.Direction.REVERSE);
        armServo.setPower(ArmConstants.back);


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
        armServo.setPower(pos);
    }

    public static void reset() {
        armServo.setPower(ArmConstants.back); // TODO or front...
    }

    public static void testArm (Gamepad gamepad, Telemetry telemetry) {
        armServo.setPower(gamepad.left_stick_y);
        telemetry.addData("pos", armServo.getPower());

    }
}
