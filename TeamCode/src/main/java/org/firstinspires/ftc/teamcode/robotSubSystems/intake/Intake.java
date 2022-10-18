package org.firstinspires.ftc.teamcode.robotSubSystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    public static final DcMotor motors[] = new DcMotor[2];
    private static float power;

    public void init(HardwareMap hardwareMap) {

        motors[0] = hardwareMap.get(DcMotor.class, "IntakeR");
        motors[1] = hardwareMap.get(DcMotor.class, "IntakeL");
        //TODO: reverse correct motor according to robot

        for (final DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void operate(IntakeState state) {
        switch (state) {
            case COLLECT:
                power = 1;
                //TODO: adjust to appropriate value for collection
                break;
            case STOP:
                power = 0;
                break;
            case DEPLETE:
                power = -1;
                //TODO: adjust to appropriate value for depletion
                break;
        }

        for (final DcMotor motor : motors) motor.setPower(power);

    }
}
