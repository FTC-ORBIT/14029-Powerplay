package org.firstinspires.ftc.teamcode.robotSubSystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    public static final DcMotor motors[] = new DcMotor[2];
    private static float power;

    public static void init(HardwareMap hardwareMap) {

        motors[0] = hardwareMap.get(DcMotor.class, "IntakeR");
        motors[1] = hardwareMap.get(DcMotor.class, "IntakeL");

        for (final DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public static void operate(IntakeState state) {
        switch (state) {
            case COLLECT:
                power = IntakeConstants.intakePower;
                break;
            case STOP:
                power = 0;
                break;
            case DEPLETE:
                power = IntakeConstants.depletePower;
                break;
        }

        for (final DcMotor motor : motors)
            motor.setPower(power);

    }
}
