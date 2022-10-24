package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants.gearRatio;
import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants.groundHeight;
import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants.highHeight;
import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants.intakeHeight;
import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants.kP;
import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants.lowHeight;
import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants.midHeight;
import static org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstants.overrideHeight;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OrbitUtils.PID;

public class Elevator {
    private static DcMotor firstMotor = null;
    private static DcMotor secondMotor = null;
    private static final DcMotor elevatorEncoder = firstMotor; //or second motor?
    private static float height;
    private static final PID elevatorPID = new PID(kP, 0, 0, 0, 0);
    private static float lastStateHeight;
    private static boolean lastOverrideState = false;
    private  static float elevatorPower = 0f;

    public static void init(HardwareMap hardwareMap){
        firstMotor = hardwareMap.get(DcMotor.class, "firstMotor");
        secondMotor = hardwareMap.get(DcMotor.class, "secondMotor");
        //TODO reverse the motors if we need to
    }


    public static void operate (ElevatorStates elevatorState, Gamepad gamepad1){
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
            case OVERRIDE:
                elevatorPower = gamepad1.right_stick_y;
                break;
        }
        if(!elevatorState.equals(ElevatorStates.OVERRIDE)){
            elevatorPower = (float) elevatorPID.update(height);
        }

        firstMotor.setPower(elevatorPower);
        secondMotor.setPower(elevatorPower);
    }

}
