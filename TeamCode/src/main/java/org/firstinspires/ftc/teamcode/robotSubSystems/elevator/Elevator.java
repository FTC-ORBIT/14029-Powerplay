package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstans.gearRatio;
import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstans.groundHeight;
import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstans.highHeight;
import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstans.intakeHeight;
import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstans.kP;
import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstans.lowHeight;
import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstans.midHeight;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OrbitUtils.PID;

public class Elevator {
    private static DcMotor firstMotor = null;
    private static DcMotor secondMotor = null;
    private static final DcMotor elevatorEncoder = firstMotor; //or second motor?
    private static float height;
    private static final PID elevatorPID = new PID(kP, 0, 0, 0, 0);

    public static void init(HardwareMap hardwareMap){
        firstMotor = hardwareMap.get(DcMotor.class, "firstMotor");
        secondMotor = hardwareMap.get(DcMotor.class, "secondMotor");
        //TODO reverse the motors if we need to
    }


    public static void operate (ElevatorStates elevatorState){
        final float height = elevatorEncoder.getCurrentPosition() * gearRatio;
        switch (elevatorState){
            case INTAKE:
                elevatorPID.setWanted(intakeHeight);
                break;
            case GROUND:
                elevatorPID.setWanted(groundHeight);
                break;
            case LOW:
                elevatorPID.setWanted(lowHeight);
                break;
            case MID:
                elevatorPID.setWanted(midHeight);
                break;
            case HIGH:
                elevatorPID.setWanted(highHeight);
                break;
        }
        firstMotor.setPower(elevatorPID.update(height));
        secondMotor.setPower(elevatorPID.update(height));
    }

}
