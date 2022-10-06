package org.firstinspires.ftc.teamcode.robotSubSystems.intake;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    public static final DcMotor motors[] = new DcMotor[2];

    public void init(HardwareMap hardwareMap) {

        motors[0] = hardwareMap.get(DcMotor.class, "IntakeR");
        motors[1] = hardwareMap.get(DcMotor.class, "IntakeL");
        //TODO: reverse correct motor according to robot

        for (final DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void operate() {

        for (final DcMotor motor : motors) {
            motor.setPower(1);
            //TODO: change power according to tests
        }

    }

    public void stop() {

    }

    public void reset() {

    }
}
